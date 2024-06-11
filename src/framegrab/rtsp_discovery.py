import logging
import onvif

from onvif import ONVIFCamera
from wsdiscovery.discovery import ThreadedWSDiscovery as WSDiscovery

logger = logging.getLogger(__name__)

DEFAULT_CREDENTIALS = [
    ("admin", "admin"),
    ("admin", ""),
    ("", ""),
    ("root", "camera"),
    ("root", "root"),
    ("admin", "12345"),
    ("admin", "123456"),
    ("admin", "password"),
    ("user", "user"),
    ("root", "pass"),
]


class RTSPDiscovery:
    # Simple RTSP camera discovery with ONVIF capabilities

    def __init__(self) -> None:
        pass

    @staticmethod
    def discover_camera_ips() -> set:
        """Use WSDiscovery to find ONVIF devices.
        Returns a list of IP addresses of ONVIF devices.
        """

        device_ips = set()
        logger.debug("Starting WSDiscovery for ONVIF devices")
        wsd = WSDiscovery()
        wsd.start()
        ret = wsd.searchServices()
        for service in ret:
            xaddr = service.getXAddrs()[0]
            # each service has a bunch of QName's == qualified names
            qnames = service.getTypes()
            for qname in qnames:
                # the qname's we care about are
                # - http://www.onvif.org/ver10/network/wsdl:NetworkVideoTransmitter
                # - http://www.onvif.org/ver10/device/wsdl:Device
                if "onvif" in str(qname):
                    logger.debug(f"Found ONVIF service {qname} at {xaddr}")
                    # xaddr will be like "http://10.44.2.95/onvif/device_service"
                    ip = xaddr.split("//")[1].split("/")[0]
                    device_ips.add(ip)
        wsd.stop()
        return list(device_ips)

    @staticmethod
    def generate_rtsp_urls(
        ip: str, username: str = "admin", password: str = "admin"
    ) -> list[str]:
        """Fetch RTSP URLs from an ONVIF device, given a username/password.
        Returns [] if the device is unreachable or the credentials are wrong.
        """

        rtsp_urls = []
        try:
            try:
                # Assuming port 80, adjust if necessary
                cam = ONVIFCamera(ip, 80, username, password)
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
            except onvif.exceptions.ONVIFError as e:
                msg = str(e).lower()
                if "auth" in msg:  # looks like a bad login - give up.
                    return []
                else:
                    raise  # something else
        except Exception as e:
            logger.error(f"Error fetching RTSP URL for {ip}: {e}", exc_info=True)

        # Now insert the username/password into the URLs
        for i, url in enumerate(rtsp_urls):
            rtsp_urls[i] = url.replace("rtsp://", f"rtsp://{username}:{password}@")
        return rtsp_urls
