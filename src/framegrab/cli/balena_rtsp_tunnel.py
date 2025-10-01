import subprocess
from pathlib import Path
import os

import click

COMMAND_NAME = "balena-rtsp-tunnel"

@click.command(COMMAND_NAME)
@click.argument("device_id")
@click.argument("rtsp_ip")
@click.argument("pem_file", required=False)
@click.option("-l", "--local-port", default=8554, help="Local port for RTSP stream")
@click.option("-r", "--remote-port", default=554, help="Remote RTSP port")
def balena_rtsp_tunnel(
    device_id: str, rtsp_ip: str, pem_file: str = None, local_port: int = 8554, remote_port: int = 554
):
    """Tunnel an RTSP stream from a Balena device via SSH tunneling

    \b
    DEVICE_ID: Balena device ID
    RTSP_IP: IP address of RTSP camera (e.g. 192.168.2.219)
    PEM_FILE: Optional path to PEM file for SSH authentication
    """
    if os.name == 'nt':
        raise RuntimeError(f"{COMMAND_NAME} is not supported on Windows. Try running this command on a Unix-like machine.")

    script_path = Path(__file__).parent / "balena_rtsp_tunnel.sh"  # Poetry packages .sh files alongside .py files, so this works
    cmd = [str(script_path), device_id, rtsp_ip, pem_file or "", str(local_port), str(remote_port)]
    subprocess.run(cmd)
