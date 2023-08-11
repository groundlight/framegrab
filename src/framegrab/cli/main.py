#!/usr/bin/env python3
import click

from . import autodiscover, preview

@click.group()
def cli():
    """Framegrab CLI."""
    pass

#@cli.add_command(autodiscover.autodiscover)
#@cli.add_command(preview.preview)

if __name__ == "__main__":
    cli()
