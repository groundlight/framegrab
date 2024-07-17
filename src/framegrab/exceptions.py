class GrabError(Exception):
    """Exception raised for errors in the frame grabbing process."""

    def __init__(self, message="Failed to grab frame from camera."):
        self.message = message
        super().__init__(self.message)
