#!/usr/bin/env python3
import click

from . import autodiscover, balena_rtsp_tunnel, preview


@click.group()
def climain():
    """Command-line interface to Framegrab library."""
    pass


climain.add_command(autodiscover.autodiscover)
climain.add_command(balena_rtsp_tunnel.balena_rtsp_tunnel)
climain.add_command(preview.preview)

if __name__ == "__main__":
    climain()
