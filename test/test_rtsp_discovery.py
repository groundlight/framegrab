import unittest

from unittest.mock import patch, MagicMock
from framegrab.rtsp_discovery import RTSPDiscovery, ONVIFDeviceInfo, AutodiscoverMode


class TestRTSPDiscovery(unittest.TestCase):
    def test_discover_camera_ips(self):
        mock_discovery_result = [
            {
                'epr': 'urn:uuid:test-device-1',
                'types': ['dn:NetworkVideoTransmitter', 'tds:Device'],
                'scopes': ['onvif://www.onvif.org/type/NetworkVideoTransmitter'],
                'xaddrs': ['http://localhost:8080/onvif/device_service'],
                'host': 'localhost',
                'port': 8080,
                'use_https': False
            }
        ]
        
        with patch(
            "framegrab.rtsp_discovery.RTSPDiscovery._get_wsd"
        ) as mock_get_wsd:
            # Mock ONVIFDiscovery instance
            mock_discovery_instance = MagicMock()
            mock_discovery_instance.discover.return_value = mock_discovery_result
            mock_get_wsd.return_value = mock_discovery_instance
            
            devices = RTSPDiscovery.discover_onvif_devices()

            assert devices[0].xaddr == "http://localhost:8080/onvif/device_service"

    def test_generate_rtsp_urls(self):
        device = ONVIFDeviceInfo(ip="0")

        assert [] == RTSPDiscovery.generate_rtsp_urls(device=device)
        assert device.rtsp_urls == []

    def test_try_logins(self):
        device = ONVIFDeviceInfo(ip="0")

        assert False == RTSPDiscovery._try_logins(device=device, auto_discover_mode=AutodiscoverMode.complete_fast)
        assert device.rtsp_urls == []
