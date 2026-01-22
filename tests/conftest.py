"""
Pytest configuration and shared fixtures for ROSBridgeML tests.
"""
import os
import pytest
import tempfile
import shutil
from pathlib import Path


@pytest.fixture
def temp_dir():
    """Create a temporary directory for test files."""
    temp_path = tempfile.mkdtemp()
    yield temp_path
    shutil.rmtree(temp_path, ignore_errors=True)


@pytest.fixture
def examples_dir():
    """Return path to examples directory."""
    return Path(__file__).parent.parent / "examples"


@pytest.fixture
def sample_ros_model():
    """Return a valid ROS model as a string."""
    return """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        odom [ "nav_msgs/Odometry", "/odom" ]
        cmd_vel [ "geometry_msgs/Twist", "/cmd_vel" ]
    
    SERVICES
        reset [ "std_srvs/Empty", "/reset" ]
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[Topic] odom_bridge odom:"robot/odom";
Bridge[Topic] cmd_bridge "robot/cmd":cmd_vel;
Bridge[Service] reset_bridge "robot/reset":reset;
"""


@pytest.fixture
def sample_ros2_model():
    """Return a valid ROS2 model as a string."""
    return """
Robot[ROS2] TestRobot2 @ "localhost" {
    TOPICS
        odom [ "nav_msgs/msg/Odometry", "/odom" ]
        cmd_vel [ "geometry_msgs/msg/Twist", "/cmd_vel" ]
    
    SERVICES
        reset [ "std_srvs/srv/Empty", "/reset" ]
    
    ACTIONS
        navigate [ "nav2_msgs/action/NavigateToPose", "/navigate_to_pose" ]
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[Topic] odom_bridge odom:"robot/odom";
Bridge[Action] nav_bridge "robot/navigate":navigate;
"""


@pytest.fixture
def sample_node_bridge_model():
    """Return a model with NodeBridge for testing expansion."""
    return """
Robot[ROS2] NodeBridgeTest @ "localhost" {
    TOPICS
        pose [ "geometry_msgs/msg/Pose", "/pose" ]
        velocity [ "geometry_msgs/msg/Twist", "/velocity" ]
        status [ "std_msgs/msg/String", "/status" ]
    
    SERVICES
        start [ "std_srvs/srv/Trigger", "/start" ]
        stop [ "std_srvs/srv/Trigger", "/stop" ]
    
    NODES
        controller {
            publishes: [ pose, status ]
            subscribes: [ velocity ]
            services: [ start, stop ]
        }
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[Node] controller_bridge controller:"robot/controller";
"""


@pytest.fixture
def invalid_model():
    """Return an invalid model (missing required sections)."""
    return """
Robot[ROS] InvalidRobot @ "localhost" {
    TOPICS
        odom [ "nav_msgs/Odometry", "/odom" ]
}
"""


@pytest.fixture
def sample_broker_config():
    """Return a broker configuration dict."""
    return {
        "host": "localhost",
        "port": 1883,
        "auth": {
            "username": "testuser",
            "password": "testpass"
        }
    }


@pytest.fixture
def metamodel():
    """Return a robocon metamodel instance."""
    from robocon.utils import get_mm
    return get_mm(debug=False, global_scope=True)


@pytest.fixture
def write_model_file(temp_dir):
    """Factory fixture to write a model string to a file."""
    def _write_model(model_str, filename="test_model.rbr"):
        filepath = os.path.join(temp_dir, filename)
        with open(filepath, 'w') as f:
            f.write(model_str)
        return filepath
    return _write_model


@pytest.fixture
def model_file(temp_dir, sample_ros_model, write_model_file):
    """Create a temporary ROS model file."""
    return write_model_file(sample_ros_model)


@pytest.fixture
def ros2_model_file(temp_dir, sample_ros2_model, write_model_file):
    """Create a temporary ROS2 model file."""
    return write_model_file(sample_ros2_model)


@pytest.fixture
def node_bridge_model_file(temp_dir, sample_node_bridge_model, write_model_file):
    """Create a temporary NodeBridge model file."""
    return write_model_file(sample_node_bridge_model)


@pytest.fixture
def syntax_error_model():
    """Return a model with syntax error (missing colon)."""
    return """
ros1

ROSTopic test_topic {
    name "/test"
    type: "std_msgs/String"
}
"""


@pytest.fixture
def semantic_error_model():
    """Return a model with semantic error (undefined reference)."""
    return """
ros1

MessageBroker mqtt {
    host: "localhost"
}

Bridge[Topic] test_bridge undefined_topic:"test/topic";
"""


@pytest.fixture
def empty_model():
    """Return an empty model string."""
    return ""

