#!/usr/bin/env python3
import click

from . import autodiscover, preview, rtsp_server


@click.group()
def climain():
    """Command-line interface to Framegrab library."""
    pass


climain.add_command(autodiscover.autodiscover)
climain.add_command(preview.preview)
climain.add_command(rtsp_server.rtsp_server)

if __name__ == "__main__":
    climain()
