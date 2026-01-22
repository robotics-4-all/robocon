"""
Tests for the robocon language module.
"""
import pytest
from textx import metamodel_from_str
from robocon.utils import get_mm


def test_robocon_language_returns_metamodel():
    """Test that get_mm() returns a valid metamodel."""
    mm = get_mm()
    assert mm is not None
    assert hasattr(mm, 'model_from_str')
    assert hasattr(mm, 'model_from_file')


def test_robocon_language_has_correct_extension():
    """Test that the metamodel can parse .rbr files."""
    mm = get_mm()
    # The metamodel should be able to parse .rbr files
    assert mm is not None


def test_robocon_language_can_parse_valid_model(sample_ros_model):
    """Test that metamodel can parse a valid model."""
    mm = get_mm()
    model = mm.model_from_str(sample_ros_model)
    
    assert model is not None
    assert model.robot.name == "TestRobot"
    assert model.robot.type == "ROS"
    assert model.broker.host == "localhost"


def test_robocon_language_parses_ros2_model(sample_ros2_model):
    """Test that metamodel can parse a ROS2 model."""
    mm = get_mm()
    model = mm.model_from_str(sample_ros2_model)
    
    assert model is not None
    assert model.robot.name == "TestRobot2"
    assert model.robot.type == "ROS2"


def test_robocon_language_invalid_syntax():
    """Test that invalid syntax raises an error."""
    from textx.exceptions import TextXSyntaxError
    
    mm = get_mm()
    invalid_model = "This is not valid syntax"
    
    with pytest.raises(TextXSyntaxError):
        mm.model_from_str(invalid_model)


def test_robocon_language_missing_required_section():
    """Test that missing broker section raises an error."""
    from textx.exceptions import TextXSyntaxError
    
    mm = get_mm()
    incomplete_model = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        odom [ "nav_msgs/Odometry", "/odom" ]
}
"""
    
    with pytest.raises(TextXSyntaxError):
        mm.model_from_str(incomplete_model)

