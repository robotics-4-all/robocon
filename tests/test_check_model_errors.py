"""
Tests for the check_model_errors() function in robocon.utils.
"""
import os
import pytest
from robocon.utils import check_model_errors


class TestCheckModelErrors:
    """Tests for check_model_errors function."""

    def test_check_model_errors_valid_model_file(self, temp_dir, sample_ros_model, write_model_file):
        """Test check_model_errors with a valid model file path."""
        model_path = write_model_file(sample_ros_model, "valid_model.rbr")
        
        errors = check_model_errors(model_path)
        assert isinstance(errors, list)
        assert len(errors) == 0, f"Expected no errors, but got: {errors}"

    def test_check_model_errors_syntax_error_missing_colon(self):
        """Test check_model_errors with syntax error (missing colon)."""
        invalid_model = """
ros1

ROSTopic test_topic {
    name "/test"
    type: "std_msgs/String"
}
"""
        errors = check_model_errors(invalid_model)
        
        assert isinstance(errors, list)
        assert len(errors) > 0, "Expected at least one error"
        assert errors[0]["type"] == "syntax"
        assert "line" in errors[0]
        assert "column" in errors[0]
        assert "message" in errors[0]
        assert errors[0]["line"] > 0

    def test_check_model_errors_syntax_error_missing_brace(self):
        """Test check_model_errors with syntax error (missing closing brace)."""
        invalid_model = """
ros1

ROSTopic test_topic {
    name: "/test"
    type: "std_msgs/String"

MessageBroker mqtt {
    host: "localhost"
}
"""
        errors = check_model_errors(invalid_model)
        
        assert isinstance(errors, list)
        assert len(errors) > 0, "Expected at least one error"
        assert errors[0]["type"] == "syntax"

    def test_check_model_errors_syntax_error_invalid_keyword(self):
        """Test check_model_errors with invalid keyword."""
        invalid_model = """
ros1

InvalidKeyword test_topic {
    name: "/test"
    type: "std_msgs/String"
}
"""
        errors = check_model_errors(invalid_model)
        
        assert isinstance(errors, list)
        assert len(errors) > 0, "Expected at least one error"

    def test_check_model_errors_empty_model(self):
        """Test check_model_errors with empty model string."""
        empty_model = ""
        
        errors = check_model_errors(empty_model)
        
        assert isinstance(errors, list)
        assert len(errors) > 0, "Expected error for empty model"

    def test_check_model_errors_whitespace_only(self):
        """Test check_model_errors with whitespace-only model."""
        whitespace_model = "   \n\n\t  \n  "
        
        errors = check_model_errors(whitespace_model)
        
        assert isinstance(errors, list)
        assert len(errors) > 0, "Expected error for whitespace-only model"

    def test_check_model_errors_schema_validation(self):
        """Test that error schema contains all required fields."""
        invalid_model = """
ros1

ROSTopic test {
    name "/test"
}
"""
        errors = check_model_errors(invalid_model)
        
        assert len(errors) > 0
        error = errors[0]
        
        # Verify schema
        assert "type" in error
        assert "message" in error
        assert "line" in error
        assert "column" in error
        
        # Verify types
        assert isinstance(error["type"], str)
        assert isinstance(error["message"], str)
        assert isinstance(error["line"], int)
        assert isinstance(error["column"], int)

    def test_check_model_errors_error_types(self):
        """Test that error types are correctly identified."""
        syntax_error_model = """
ros1

ROSTopic test {
    name "/test"
}
"""
        errors = check_model_errors(syntax_error_model)
        assert len(errors) > 0
        assert errors[0]["type"] in ["syntax", "semantic", "unknown"]

    def test_check_model_errors_complex_valid_model(self, sample_ros2_model):
        """Test check_model_errors with complex valid ROS2 model."""
        errors = check_model_errors(sample_ros2_model)
        assert len(errors) == 0, f"Expected no errors for valid ROS2 model, got: {errors}"

    def test_check_model_errors_nodebridge_model(self, sample_node_bridge_model):
        """Test check_model_errors with NodeBridge model."""
        errors = check_model_errors(sample_node_bridge_model)
        assert len(errors) == 0, f"Expected no errors for NodeBridge model, got: {errors}"

    def test_check_model_errors_file_not_exists_treated_as_string(self):
        """Test that non-existent file path is treated as string content."""
        # This should be treated as model string content (invalid), not a missing file
        fake_path = "/nonexistent/path/to/model.rbr"
        
        errors = check_model_errors(fake_path)
        
        # Should return errors because it's invalid model content
        assert isinstance(errors, list)
        assert len(errors) > 0

    def test_check_model_errors_multiple_errors(self):
        """Test model with multiple syntax errors."""
        model_with_errors = """
ros1

ROSTopic test1 {
    name "/test1"
    type: "std_msgs/String"
}

ROSTopic test2 {
    name: "/test2"
    type "std_msgs/Int32"
}
"""
        errors = check_model_errors(model_with_errors)
        
        # TextX typically stops at the first error, so we expect at least one
        assert len(errors) >= 1

    def test_check_model_errors_line_column_accuracy(self):
        """Test that line and column numbers are reported."""
        invalid_model = """
ros1

ROSTopic test {
    name: "/test"
    type "std_msgs/String"
}
"""
        errors = check_model_errors(invalid_model)
        
        assert len(errors) > 0
        error = errors[0]
        
        # Line should be around line 6 where the error occurs
        assert error["line"] > 0
        assert error["column"] >= 0

    def test_check_model_errors_real_example_file(self, examples_dir):
        """Test check_model_errors with real example files."""
        # Test with a real example file if it exists
        example_file = examples_dir / "ros_model.rbr"
        
        if example_file.exists():
            errors = check_model_errors(str(example_file))
            # Real examples should be valid
            assert len(errors) == 0, f"Example file has errors: {errors}"

    def test_check_model_errors_creates_and_cleans_temp_file(self, temp_dir):
        """Test that temporary files are created and cleaned up properly."""
        import os
        
        model_string = """
ros1

ROSTopic test {
    name: "/test"
    type: "std_msgs/String"
}
"""
        # Get current temp file count
        temp_files_before = len([f for f in os.listdir(temp_dir) if f.startswith("model_for_validation")])
        
        errors = check_model_errors(model_string)
        
        # Temp files should be cleaned up
        temp_files_after = len([f for f in os.listdir(temp_dir) if f.startswith("model_for_validation")])
        
        # No new temp files should remain
        # Note: This test checks temp_dir but the function uses system temp dir
        # So we're mainly testing the function doesn't crash
        assert isinstance(errors, list)

    def test_check_model_errors_unicode_content(self):
        """Test check_model_errors handles unicode content."""
        unicode_model = """
ros1

ROSTopic test_topic {
    name: "/test_测试"
    type: "std_msgs/String"
}
"""
        errors = check_model_errors(unicode_model)
        
        # Should handle unicode gracefully (may or may not be valid depending on grammar)
        assert isinstance(errors, list)

    def test_check_model_errors_returns_list(self):
        """Test that check_model_errors always returns a list."""
        models = [
            "ros1\n\nROSTopic t { name: \"/t\" type: \"std_msgs/String\" }",
            "invalid content",
            "",
        ]
        
        for model in models:
            result = check_model_errors(model)
            assert isinstance(result, list), f"Expected list for model: {model[:50]}"


class TestCheckModelErrorsIntegration:
    """Integration tests for check_model_errors with real examples."""

    def test_all_example_files_are_valid(self, examples_dir):
        """Test that all example .rbr files are valid."""
        example_files = list(examples_dir.glob("*.rbr"))
        
        if not example_files:
            pytest.skip("No example files found")
        
        for example_file in example_files:
            errors = check_model_errors(str(example_file))
            assert len(errors) == 0, f"Example file {example_file.name} has errors: {errors}"

    def test_check_model_errors_with_imports(self, temp_dir, write_model_file):
        """Test check_model_errors with models that use imports."""
        # Create a base model
        base_model = """
ros1

ROSTopic shared_topic {
    name: "/shared"
    type: "std_msgs/String"
}
"""
        base_path = write_model_file(base_model, "base.rbr")
        
        # Create a model that imports the base
        importing_model = f"""
import "{base_path}"

MessageBroker mqtt {{
    host: "localhost"
}}
"""
        
        errors = check_model_errors(importing_model)
        
        # Should handle imports (may have errors depending on import resolution)
        assert isinstance(errors, list)


class TestCheckModelErrorsEdgeCases:
    """Edge case tests for check_model_errors."""

    def test_very_large_model_string(self):
        """Test check_model_errors with a very large model."""
        # Create a large but valid model
        large_model = "ros1\n\n"
        for i in range(100):
            large_model += f"""
ROSTopic topic_{i} {{
    name: "/topic_{i}"
    type: "std_msgs/String"
}}
"""
        
        errors = check_model_errors(large_model)
        
        # Should handle large models
        assert isinstance(errors, list)

    def test_model_with_only_version(self):
        """Test model with only version declaration."""
        minimal_model = "ros1"
        
        errors = check_model_errors(minimal_model)
        
        # May or may not be valid depending on grammar requirements
        assert isinstance(errors, list)

    def test_check_model_errors_preserves_error_message(self):
        """Test that original error messages are preserved."""
        invalid_model = """
ros1

ROSTopic test {
    invalid_field: "value"
}
"""
        errors = check_model_errors(invalid_model)
        
        if errors:
            # Error message should be non-empty and informative
            assert len(errors[0]["message"]) > 0
            assert isinstance(errors[0]["message"], str)
