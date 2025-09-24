import unittest
from typing import Optional

from framegrab.config import FrameGrabberConfig, OptionsField, InputTypes


class OptionsFieldTestConfig(FrameGrabberConfig):
    """Test configuration class with various OptionsField types for testing."""
    
    # Simple scalar option
    simple_value: OptionsField[int] = OptionsField(key="simple_value", default=42)
    
    # Nested option with dot-path
    nested_setting: OptionsField[str | None] = OptionsField(key="camera.setting", default=None)
    
    # Deeply nested option
    deep_option: OptionsField[bool] = OptionsField(key="advanced.features.enabled", default=False)
    
    # Optional field that can be None
    optional_param: OptionsField[float | None] = OptionsField(key="optional_param", default=None)
    
    # Test ID field (not an OptionsField) - use serial_number which MockFrameGrabberConfig expects
    serial_number: Optional[str] = None

    @classmethod
    def get_input_type(cls):
        """Override to provide a test input type."""
        return InputTypes.MOCK

    @classmethod
    def get_input_type_to_class_dict(cls):
        """Override to include our test class, replacing MockFrameGrabberConfig."""
        base_dict = super().get_input_type_to_class_dict()
        base_dict[InputTypes.MOCK] = cls  # Replace MockFrameGrabberConfig with our test config
        return base_dict


class TestOptionsField(unittest.TestCase):
    """Test cases for OptionsField functionality."""

    def test_options_field_round_trip_with_defaults(self):
        """Test that OptionsField values round-trip correctly with default values."""
        config = OptionsFieldTestConfig()
        
        # Check initial values
        self.assertEqual(config.simple_value, 42)
        self.assertIsNone(config.nested_setting)
        self.assertEqual(config.deep_option, False)
        self.assertIsNone(config.optional_param)
        
        # Convert to dict
        config_dict = config.to_framegrab_config_dict()
        
        # Check structure
        self.assertEqual(config_dict["input_type"], "mock")
        # No ID section when serial_number is None
        
        options = config_dict["options"]
        self.assertEqual(options["simple_value"], 42)
        self.assertNotIn("nested_setting", str(options))  # None values should be omitted
        self.assertEqual(options["advanced"]["features"]["enabled"], False)
        self.assertNotIn("optional_param", str(options))  # None values should be omitted
        
        # Round-trip back to config
        restored_config = OptionsFieldTestConfig.from_framegrab_config_dict(config_dict)
        
        # Verify restoration
        self.assertIsInstance(restored_config, OptionsFieldTestConfig)
        self.assertEqual(restored_config.simple_value, 42)
        self.assertIsNone(restored_config.nested_setting)
        self.assertEqual(restored_config.deep_option, False)
        self.assertIsNone(restored_config.optional_param)
        self.assertIsNone(restored_config.serial_number)

    def test_options_field_round_trip_with_custom_values(self):
        """Test that OptionsField values round-trip correctly with custom values."""
        config = OptionsFieldTestConfig(
            simple_value=100,
            nested_setting="custom_setting",
            deep_option=True,
            optional_param=3.14,
            serial_number="custom_device"
        )
        
        # Convert to dict
        config_dict = config.to_framegrab_config_dict()
        
        # Check structure
        options = config_dict["options"]
        self.assertEqual(options["simple_value"], 100)
        self.assertEqual(options["camera"]["setting"], "custom_setting")
        self.assertEqual(options["advanced"]["features"]["enabled"], True)
        self.assertEqual(options["optional_param"], 3.14)
        
        # Round-trip back to config
        restored_config = OptionsFieldTestConfig.from_framegrab_config_dict(config_dict)
        
        # Verify restoration
        self.assertEqual(restored_config.simple_value, 100)
        self.assertEqual(restored_config.nested_setting, "custom_setting")
        self.assertEqual(restored_config.deep_option, True)
        self.assertEqual(restored_config.optional_param, 3.14)
        self.assertEqual(restored_config.serial_number, "custom_device")

    def test_options_field_from_dict_creation(self):
        """Test creating config from dictionary with nested options."""
        config_dict = {
            "input_type": "mock",
            "name": "test_config",
            "id": {"serial_number": "dict_device"},
            "options": {
                "simple_value": 200,
                "camera": {
                    "setting": "from_dict"
                },
                "advanced": {
                    "features": {
                        "enabled": True
                    }
                },
                "optional_param": 2.71
            }
        }
        
        config = OptionsFieldTestConfig.from_framegrab_config_dict(config_dict)
        
        # Verify all values were extracted correctly
        self.assertIsInstance(config, OptionsFieldTestConfig)
        self.assertEqual(config.simple_value, 200)
        self.assertEqual(config.nested_setting, "from_dict")
        self.assertEqual(config.deep_option, True)
        self.assertEqual(config.optional_param, 2.71)
        self.assertEqual(config.serial_number, "dict_device")
        self.assertEqual(config.name, "test_config")

    def test_unknown_options_raise_error(self):
        """Test that unknown option keys raise a ValueError."""
        config_dict = {
            "input_type": "mock",
            "id": {"serial_number": "test_device"},
            "options": {
                "simple_value": 100,
                "unknown_option": "should_fail",
                "another": {
                    "unknown": "nested_option"
                }
            }
        }
        
        with self.assertRaises(ValueError) as context:
            OptionsFieldTestConfig.from_framegrab_config_dict(config_dict)
        
        error_message = str(context.exception)
        self.assertIn("Unexpected option keys", error_message)
        self.assertIn("OptionsFieldTestConfig", error_message)
        # Should mention at least one of the unknown keys
        self.assertTrue("unknown_option" in error_message or "another" in error_message)

    def test_partial_options_extraction(self):
        """Test that only some options can be provided while others use defaults."""
        config_dict = {
            "input_type": "mock",
            "id": {"serial_number": "partial_device"},
            "options": {
                "camera": {
                    "setting": "only_nested"
                }
                # simple_value, deep_option, optional_param not provided - should use defaults
            }
        }
        
        config = OptionsFieldTestConfig.from_framegrab_config_dict(config_dict)
        
        # Check that provided option was extracted
        self.assertEqual(config.nested_setting, "only_nested")
        
        # Check that defaults were used for non-provided options
        self.assertEqual(config.simple_value, 42)  # default
        self.assertEqual(config.deep_option, False)  # default
        self.assertIsNone(config.optional_param)  # default

    def test_empty_options_uses_all_defaults(self):
        """Test that empty options dict results in all default values."""
        config_dict = {
            "input_type": "mock",
            "id": {"serial_number": "empty_options_device"},
            "options": {}
        }
        
        config = OptionsFieldTestConfig.from_framegrab_config_dict(config_dict)
        
        # All should be defaults
        self.assertEqual(config.simple_value, 42)
        self.assertIsNone(config.nested_setting)
        self.assertEqual(config.deep_option, False)
        self.assertIsNone(config.optional_param)
        self.assertEqual(config.serial_number, "empty_options_device")
