import time
import onvif
import logging
import urllib.parse

from enum import Enum
from wsdiscovery import QName
from onvif import ONVIFCamera
from pydantic import BaseModel
from typing import List, Optional, Union
from wsdiscovery.discovery import ThreadedWSDiscovery as WSDiscovery

logger = logging.getLogger(__name__)


# Default credentials to try when connecting to RTSP cameras, used in discover_camera_ips() when try_default_logins=True
DEFAULT_CREDENTIALS = [
    ("admin", "admin"),
    ("", ""),
    ("admin", ""),
    ("root", "camera"),
    ("root", "root"),
    ("admin", "12345"),
    ("admin", "123456"),
    ("admin", "password"),
    ("user", "user"),
    ("root", "pass"),
]

"""
Enum for camera discovery modes. Options to try different default credentials stored in DEFAULT_CREDENTIALS. 
Consists of three options:
    light: only try first three usernames and passwords ("admin:admin", "admin:", and no username/password).
    complete_fast: try the entire DEFAULT_CREDENTIALS without delays in between. 
    complete_slow: try the entire DEFAULT_CREDENTIALS with a delay of 1 seconds in between.
    Defaults to None.
"""


class AutoDiscoverModes(str, Enum):
    light = "light"
    complete_fast = "complete_fast"
    complete_slow = "complete_slow"


# Model for storing ONVIF Device Information
class ONVIFDeviceInfo(BaseModel):
    ip: str
    port: Optional[int] = 80
    username: Optional[str] = ""
    password: Optional[str] = ""
    xaddr: Optional[str] = ""  # ONVIF service address
    rtsp_urls: Optional[List[str]] = []  # List of rtsp urls for the camera


class RTSPDiscovery:
    """Simple RTSP camera discovery with ONVIF capabilities"""

    @staticmethod
    def discover_camera_ips(auto_discover_modes: Union[AutoDiscoverModes, None] = None) -> List[ONVIFDeviceInfo]:
        """
        Uses WSDiscovery to find ONVIF supported devices.

        Parameters:
        auto_discover_modes (AutoDiscoverModes | None, optional): Options to try different default credentials stored in DEFAULT_CREDENTIALS.
        Consists of three options:
            light: only try first two usernames and passwords ("admin:admin" and no username/password).
            complete_fast: try the entire DEFAULT_CREDENTIALS without delays in between.
            complete_slow: try the entire DEFAULT_CREDENTIALS with a delay of 5 seconds in between.
            Defaults to None.

        Returns:
        List[ONVIFDeviceInfo]: A list of ONVIFDeviceInfos with IP address, port number, and ONVIF service address.
        """

        device_ips = []
        logger.debug("Starting WSDiscovery for ONVIF devices")
        wsd = WSDiscovery()
        wsd.start()
        types = [QName("http://www.onvif.org/ver10/network/wsdl", "NetworkVideoTransmitter")]
        ret = wsd.searchServices(types=types)
        for service in ret:
            xaddr = service.getXAddrs()[0]
            parsed_url = urllib.parse.urlparse(xaddr)
            ip = parsed_url.hostname
            port = parsed_url.port or 80  # Use the default port 80 if not specified

            logger.debug(f"Found ONVIF service at {xaddr}")
            device_ip = ONVIFDeviceInfo(ip=ip, port=port, username="", password="", xaddr=xaddr, rtsp_urls=[])

            if auto_discover_modes is not None:
                RTSPDiscovery._try_logins(device=device_ip, auto_discover_modes=auto_discover_modes)

            device_ips.append(device_ip)
        wsd.stop()
        return device_ips

    @staticmethod
    def generate_rtsp_urls(device: ONVIFDeviceInfo) -> List[str]:
        """
        Fetch RTSP URLs from an ONVIF supported device, given a username/password.

        Parameters:
        device (ONVIFDeviceInfo): Pydantic Model that stores information about camera RTSP address, port number, username, and password.

        Returns:
        List[str]: A list of RTSP URLs, empty list if error fetching URLs or incorrect credentials.
        """

        rtsp_urls = []
        try:
            try:
                # Assuming port 80, adjust if necessary
                cam = ONVIFCamera(device.ip, device.port, device.username, device.password)
                # Create media service
                media_service = cam.create_media_service()
                # Get profiles
                profiles = media_service.GetProfiles()
                stream_setup = {
                    "Stream": "RTP-Unicast",  # Specify the type of stream
                    "Transport": {"Protocol": "RTSP"},
                }

                # For each profile, get the RTSP URL
                for profile in profiles:
                    stream_uri = media_service.GetStreamUri(
                        {"StreamSetup": stream_setup, "ProfileToken": profile.token}
                    )
                    rtsp_urls.append(stream_uri.Uri)
            except onvif.ONVIFError as e:
                msg = str(e).lower()
                if "auth" in msg:  # looks like a bad login - give up.
                    return rtsp_urls
                else:
                    raise onvif.ONVIFError
        except Exception as e:
            logger.error(f"Error fetching RTSP URL for {device.ip}: {e}", exc_info=True)
            return rtsp_urls

        # Now insert the username/password into the URLs
        for i, url in enumerate(rtsp_urls):
            rtsp_urls[i] = url.replace("rtsp://", f"rtsp://{device.username}:{device.password}@")

        device.rtsp_urls = rtsp_urls
        return rtsp_urls

    def _try_logins(device: ONVIFDeviceInfo, auto_discover_modes: AutoDiscoverModes) -> bool:
        """
        Fetch RTSP URLs from an ONVIF supported device, given a username/password.

        Parameters:
        device (ONVIFDeviceInfo): Pydantic Model that stores information about camera RTSP address, port number, username, and password.
        auto_discover_modes (AutoDiscoverModes | None, optional): Options to try different default credentials stored in DEFAULT_CREDENTIALS.
        Consists of three options:
            light: only try first two usernames and passwords ("admin:admin" and no username/password).
            complete_fast: try the entire DEFAULT_CREDENTIALS without delays in between.
            complete_slow: try the entire DEFAULT_CREDENTIALS with a delay of 1 seconds in between.

        Returns:
        bool: False if the device is unreachable or the credentials are wrong, else returns True and updates ONVIFDeviceInfo with updated rtsp_urls.
        """

        credentials = DEFAULT_CREDENTIALS

        if auto_discover_modes == AutoDiscoverModes.light:
            credentials = DEFAULT_CREDENTIALS[:2]

        for username, password in credentials:
            logger.debug(f"Trying {username}:{password} for device IP {device.ip}")

            device.username = username
            device.password = password

            # Try generate rtsp urls for that device, if username or password incorrect try next
            if RTSPDiscovery.generate_rtsp_urls(device=device):
                logger.debug(f"RTSP URL fetched successfully with {username}:{password} for device IP {device.ip}")
                return True

            if auto_discover_modes == AutoDiscoverModes.complete_slow:
                time.sleep(1)

        # Return False when there are no correct credentials
        logger.debug(f"Unable to find RTSP URLs for device IP {device.ip}")
        return False
