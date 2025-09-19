import unittest
from pydantic import ValidationError

from framegrab.config import FileStreamFrameGrabberConfig, SUPPORTED_VIDEO_EXTENSIONS

class TestFrameGrabberConfig(unittest.TestCase):
    """Test cases for FrameGrabber configuration classes."""

    def test_file_stream_config_creation_valid(self):
        """Test creation of FileStreamFrameGrabberConfig with valid filenames."""
        
        # Test various valid video file formats
        valid_filenames = [f"file.{ext}" for ext in SUPPORTED_VIDEO_EXTENSIONS]
        valid_filenames += [
            'snake_case.mp4',
            'kebab-case.mp4',
            'Video with Spaces.mp4',
            '/home/user/video.mp4',    # full path
            'video/my_video.mp4',      # relative path
            'video.MP4',               # should be case insensitive
        ]
        
        for filename in valid_filenames:
            config = FileStreamFrameGrabberConfig(filename=filename)
            self.assertEqual(config.filename, filename)

    def test_file_stream_config_creation_invalid(self):
        """Test FileStreamFrameGrabberConfig validation fails for invalid filenames."""
        
        invalid_filenames = [
            "document.txt",      # Wrong extension
            "video",             # No extension
            "video.",            # Empty extension
            "video.mp44",
        ]
        
        for filename in invalid_filenames:
            with self.assertRaises(ValidationError) as context:
                FileStreamFrameGrabberConfig(filename=filename)
            

if __name__ == "__main__":
    unittest.main()
