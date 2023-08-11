import click

@click.command()
def preview():
    """Previews images from the configured cameras."""
    print("Executing preview...")