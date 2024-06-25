import unittest

from wsdiscovery.service import Service
from unittest.mock import patch
from framegrab.rtsp_discovery import RTSPDiscovery, ONVIFDeviceInfo, AutodiscoverModes


class TestRTSPDiscovery(unittest.TestCase):
    def test_discover_camera_ips(self):
        service = Service(
            types="", scopes="", xAddrs=["http://localhost:8080"], epr="", instanceId=""
        )
        with patch(
            "wsdiscovery.discovery.ThreadedWSDiscovery.searchServices",
            return_value=[service],
        ) as mock_camera_ips:
            devices = RTSPDiscovery.discover_onvif_devices()

            assert devices[0].xaddr == "http://localhost:8080"

    def test_generate_rtsp_urls(self):
        device = ONVIFDeviceInfo(ip="0")

        assert [] == RTSPDiscovery.generate_rtsp_urls(device=device)
        assert device.rtsp_urls == []

    def test_try_logins(self):
        device = ONVIFDeviceInfo(ip="0")

        assert False == RTSPDiscovery._try_logins(device=device, auto_discover_modes=AutodiscoverModes.complete_fast)
        assert device.rtsp_urls == []
