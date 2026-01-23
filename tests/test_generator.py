"""
Tests for the generator module (entry points).
"""
import pytest
from unittest.mock import Mock, patch, MagicMock


def test_generator_ros_descriptor():
    """Test that generator_ros is properly configured."""
    from robocon.generator import generator_ros
    
    assert generator_ros.language == 'robocon'
    assert generator_ros.target == 'ros'
    assert generator_ros.description == 'ROS-to-Broker communication bridges'
    assert generator_ros.generator is not None


def test_generator_ros2_descriptor():
    """Test that generator_ros2 is properly configured."""
    from robocon.generator import generator_ros2
    
    assert generator_ros2.language == 'robocon'
    assert generator_ros2.target == 'ros2'
    assert generator_ros2.description == 'ROS2-to-Broker communication bridges'
    assert generator_ros2.generator is not None


@patch('robocon.generator.GeneratorROS.generate')
def test_generator_ros_impl_calls_generator(mock_generate):
    """Test that _generator_ros_impl calls GeneratorROS.generate."""
    from robocon.generator import _generator_ros_impl
    
    mock_model = Mock()
    mock_model._tx_filename = "/path/to/model.rbr"
    mock_metamodel = Mock()
    
    _generator_ros_impl(mock_metamodel, mock_model, "/output", False, False)
    
    mock_generate.assert_called_once()


@patch('robocon.generator.GeneratorROS2.generate')
def test_generator_ros2_impl_calls_generator(mock_generate):
    """Test that _generator_ros2_impl calls GeneratorROS2.generate."""
    from robocon.generator import _generator_ros2_impl
    
    mock_model = Mock()
    mock_model._tx_filename = "/path/to/model.rbr"
    mock_metamodel = Mock()
    
    _generator_ros2_impl(mock_metamodel, mock_model, "/output", False, False)
    
    mock_generate.assert_called_once()


def test_generator_ros2_report_with_tfbridge():
    """Test GeneratorROS2.report() with TFBridge."""
    from robocon.m2t.ros2gen import GeneratorROS2
    
    mock_model = Mock()
    mock_model.robot.name = "TestRobot"
    mock_model.broker.host = "localhost"
    mock_model.broker.port = 6379
    
    # Create a mock TFBridge
    mock_tf_bridge = Mock()
    mock_tf_bridge.__class__.__name__ = 'TFBridge'
    mock_tf_bridge.prefix = "robot/tf"
    
    mock_model.bridges = [mock_tf_bridge]
    
    # Just ensure it doesn't crash
    GeneratorROS2.report(mock_model)


def test_generator_ros2_report_with_topic_bridge():
    """Test GeneratorROS2.report() with TopicBridge."""
    from robocon.m2t.ros2gen import GeneratorROS2
    
    mock_model = Mock()
    mock_model.robot.name = "TestRobot"
    mock_model.broker.host = "localhost"
    mock_model.broker.port = 6379
    
    # Create a mock TopicBridge
    mock_topic_bridge = Mock()
    mock_topic_bridge.__class__.__name__ = 'TopicBridge'
    mock_topic_bridge.topic.uri = "/scan"
    mock_topic_bridge.direction = "R2B"
    mock_topic_bridge.brokerURI = "sensors/scan"
    
    mock_model.bridges = [mock_topic_bridge]
    
    GeneratorROS2.report(mock_model)


def test_generator_ros_report_with_tfbridge():
    """Test GeneratorROS.report() with TFBridge."""
    from robocon.m2t.rosgen import GeneratorROS
    
    mock_model = Mock()
    mock_model.robot.name = "TestRobot"
    mock_model.broker.host = "localhost"
    mock_model.broker.port = 6379
    
    # Create a mock TFBridge
    mock_tf_bridge = Mock()
    mock_tf_bridge.__class__.__name__ = 'TFBridge'
    mock_tf_bridge.prefix = "robot/tf"
    
    mock_model.bridges = [mock_tf_bridge]
    
    GeneratorROS.report(mock_model)
