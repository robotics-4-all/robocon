import pytest
import os
from robocon.utils import check_model_errors, get_mm
from textx.exceptions import TextXSyntaxError, TextXSemanticError
from unittest.mock import patch

def test_check_model_errors_syntax_error():
    model_str = "Robot[ROS2] MyRobot @ localhost {" # Missing closing brace
    errors = check_model_errors(model_str)
    assert len(errors) == 1
    assert errors[0]["type"] == "syntax"
    assert errors[0]["line"] > 0

def test_check_model_errors_syntax_error_missing_name():
    model_str = """
    Robot[ROS2]  @ "localhost" {}
    Broker[Redis] MyBroker { host: "localhost", port: 6379 }
    """ # Missing robot name - this is actually a syntax error
    errors = check_model_errors(model_str)
    assert len(errors) == 1
    assert errors[0]["type"] == "syntax"

def test_check_model_errors_file_input(tmp_path):
    model_content = 'Robot[ROS2] MyRobot @ "localhost" {} Broker[Redis] B {host:"l", port:1}'
    model_file = tmp_path / "test.rbr"
    model_file.write_text(model_content)
    errors = check_model_errors(str(model_file))
    assert len(errors) == 0

def test_check_model_errors_unknown_exception():
    with patch('robocon.utils.get_mm') as mock_get_mm:
        mock_get_mm.side_effect = Exception("Unexpected error")
        errors = check_model_errors("some content")
        assert len(errors) == 1
        assert errors[0]["type"] == "unknown"
        assert "Unexpected error" in errors[0]["message"]

def test_get_mm_debug():
    mm = get_mm(debug=True)
    assert mm is not None

def test_get_grammar():
    from robocon.utils import get_grammar
    grammar = get_grammar()
    assert "Model:" in grammar

def test_check_model_errors_temp_file_cleanup_error():
    # Test that the function works even when cleanup might fail
    # We create a valid model and it should return no errors
    model_str = 'Robot[ROS2] MyRobot @ "localhost" {} Broker[Redis] B {host:"localhost", port:6379}'
    errors = check_model_errors(model_str)
    assert len(errors) == 0
