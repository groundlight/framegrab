import logging

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
