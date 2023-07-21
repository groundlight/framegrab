class UnavailableModule:
    """Useful for optional dependencies. If an optional dependency fails to be
    imported, create an UnavailableModule instance and use it as a placeholder.
    Attempting to do anything with the UnavailableModule instance will raise
    the original exception.

    In this way, we don't bother the user about optional dependencies failing
    to import until it becomes relevant.
    """

    def __init__(self, e: Exception):
        self.e = e  # save the original exception for later

    def __getattr__(self, name):
        """Raise the original exception when the user tries to do anything with
        an instance of this class.
        """
        raise self.e
