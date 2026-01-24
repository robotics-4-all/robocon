"""
Tests for ROS URI validation semantic checks.

Ensures that all ROS topic, service, and action URIs follow ROS naming conventions.
"""
import pytest
from robocon.utils import get_mm
from textx import TextXSemanticError


class TestValidRosURIs:
    """Tests for valid ROS URIs that should pass validation."""
    
    def test_simple_topic_uri(self):
        """Test simple valid topic URI."""
        model_str = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        test [ "std_msgs/String", "/test" ]
}
Broker[MQTT] TestBroker { host: "localhost", port: 1883 }
"""
        mm = get_mm()
        model = mm.model_from_str(model_str)
        assert model.robot.topics[0].uri == "/test"
    
    def test_nested_topic_uri(self):
        """Test nested topic URI with multiple segments."""
        model_str = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        test [ "std_msgs/String", "/robot/sensors/camera" ]
}
Broker[MQTT] TestBroker { host: "localhost", port: 1883 }
"""
        mm = get_mm()
        model = mm.model_from_str(model_str)
        assert model.robot.topics[0].uri == "/robot/sensors/camera"
    
    def test_uri_with_underscores(self):
        """Test URI with underscores."""
        model_str = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        test [ "std_msgs/String", "/cmd_vel" ]
        test2 [ "std_msgs/String", "/motor_speed_left" ]
}
Broker[MQTT] TestBroker { host: "localhost", port: 1883 }
"""
        mm = get_mm()
        model = mm.model_from_str(model_str)
        assert len(model.robot.topics) == 2
    
    def test_uri_with_numbers(self):
        """Test URI with numbers (not at segment start)."""
        model_str = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        test [ "std_msgs/String", "/robot1/sensor2" ]
}
Broker[MQTT] TestBroker { host: "localhost", port: 1883 }
"""
        mm = get_mm()
        model = mm.model_from_str(model_str)
        assert model.robot.topics[0].uri == "/robot1/sensor2"
    
    def test_service_uri_validation(self):
        """Test that service URIs are validated."""
        model_str = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
    SERVICES
        test_service [ "std_srvs/Empty", "/test_service" ]
}
Broker[MQTT] TestBroker { host: "localhost", port: 1883 }
"""
        mm = get_mm()
        model = mm.model_from_str(model_str)
        assert model.robot.services[0].uri == "/test_service"
    
    def test_action_uri_validation(self):
        """Test that action URIs are validated."""
        model_str = """
Robot[ROS2] TestRobot @ "localhost" {
    TOPICS
    ACTIONS
        test_action [ "nav2_msgs/action/NavigateToPose", "/navigate_to_pose" ]
}
Broker[MQTT] TestBroker { host: "localhost", port: 1883 }
"""
        mm = get_mm()
        model = mm.model_from_str(model_str)
        assert model.robot.actions[0].uri == "/navigate_to_pose"


class TestInvalidRosURIs:
    """Tests for invalid ROS URIs that should fail validation."""
    
    def test_uri_without_leading_slash(self):
        """Test that URI without leading slash is rejected."""
        model_str = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        test [ "std_msgs/String", "test" ]
}
Broker[MQTT] TestBroker { host: "localhost", port: 1883 }
"""
        mm = get_mm()
        with pytest.raises(TextXSemanticError, match="must start with '/'"):
            mm.model_from_str(model_str)
    
    def test_uri_ending_with_slash(self):
        """Test that URI ending with slash is rejected."""
        model_str = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        test [ "std_msgs/String", "/test/" ]
}
Broker[MQTT] TestBroker { host: "localhost", port: 1883 }
"""
        mm = get_mm()
        with pytest.raises(TextXSemanticError, match="cannot end with '/'"):
            mm.model_from_str(model_str)
    
    def test_uri_with_consecutive_slashes(self):
        """Test that URI with consecutive slashes is rejected."""
        model_str = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        test [ "std_msgs/String", "/robot//sensor" ]
}
Broker[MQTT] TestBroker { host: "localhost", port: 1883 }
"""
        mm = get_mm()
        with pytest.raises(TextXSemanticError, match="cannot contain consecutive '/'"):
            mm.model_from_str(model_str)
    
    def test_uri_with_spaces(self):
        """Test that URI with spaces is rejected."""
        model_str = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        test [ "std_msgs/String", "/test topic" ]
}
Broker[MQTT] TestBroker { host: "localhost", port: 1883 }
"""
        mm = get_mm()
        with pytest.raises(TextXSemanticError, match="contains invalid characters"):
            mm.model_from_str(model_str)
    
    def test_uri_with_special_characters(self):
        """Test that URI with special characters is rejected."""
        model_str = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        test [ "std_msgs/String", "/test@topic" ]
}
Broker[MQTT] TestBroker { host: "localhost", port: 1883 }
"""
        mm = get_mm()
        with pytest.raises(TextXSemanticError, match="contains invalid characters"):
            mm.model_from_str(model_str)
    
    def test_uri_with_dash(self):
        """Test that URI with dash is rejected (not allowed in ROS)."""
        model_str = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        test [ "std_msgs/String", "/test-topic" ]
}
Broker[MQTT] TestBroker { host: "localhost", port: 1883 }
"""
        mm = get_mm()
        with pytest.raises(TextXSemanticError, match="contains invalid characters"):
            mm.model_from_str(model_str)
    
    def test_uri_segment_starting_with_number(self):
        """Test that URI segment starting with number is rejected."""
        model_str = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        test [ "std_msgs/String", "/robot/1sensor" ]
}
Broker[MQTT] TestBroker { host: "localhost", port: 1883 }
"""
        mm = get_mm()
        with pytest.raises(TextXSemanticError, match="must start with a letter or underscore"):
            mm.model_from_str(model_str)
    
    def test_invalid_service_uri(self):
        """Test that invalid service URI is rejected."""
        model_str = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
    SERVICES
        test [ "std_srvs/Empty", "invalid_service" ]
}
Broker[MQTT] TestBroker { host: "localhost", port: 1883 }
"""
        mm = get_mm()
        with pytest.raises(TextXSemanticError, match="service.*must start with '/'"):
            mm.model_from_str(model_str)
    
    def test_invalid_action_uri(self):
        """Test that invalid action URI is rejected."""
        model_str = """
Robot[ROS2] TestRobot @ "localhost" {
    TOPICS
    ACTIONS
        test [ "nav2_msgs/action/NavigateToPose", "/navigate/" ]
}
Broker[MQTT] TestBroker { host: "localhost", port: 1883 }
"""
        mm = get_mm()
        with pytest.raises(TextXSemanticError, match="action.*cannot end with '/'"):
            mm.model_from_str(model_str)


class TestEdgeCases:
    """Tests for edge cases in ROS URI validation."""
    
    def test_uri_starting_with_underscore(self):
        """Test URI segment starting with underscore (valid)."""
        model_str = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        test [ "std_msgs/String", "/_private/topic" ]
}
Broker[MQTT] TestBroker { host: "localhost", port: 1883 }
"""
        mm = get_mm()
        model = mm.model_from_str(model_str)
        assert model.robot.topics[0].uri == "/_private/topic"
    
    def test_very_nested_uri(self):
        """Test deeply nested URI."""
        model_str = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        test [ "std_msgs/String", "/a/b/c/d/e/f/g" ]
}
Broker[MQTT] TestBroker { host: "localhost", port: 1883 }
"""
        mm = get_mm()
        model = mm.model_from_str(model_str)
        assert model.robot.topics[0].uri == "/a/b/c/d/e/f/g"
    
    def test_single_character_segments(self):
        """Test URI with single character segments."""
        model_str = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        test [ "std_msgs/String", "/a/b/c" ]
}
Broker[MQTT] TestBroker { host: "localhost", port: 1883 }
"""
        mm = get_mm()
        model = mm.model_from_str(model_str)
        assert model.robot.topics[0].uri == "/a/b/c"
