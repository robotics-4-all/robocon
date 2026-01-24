"""
Tests for verifying that generated code uses correct prefixes and topic mappings
for all bridge types (TFBridge, NodeBridge, TopicBridge, ServiceBridge, ActionBridge).
"""
import pytest
import tempfile
from pathlib import Path
from robocon.utils import build_model
from robocon.m2t.rosgen import GeneratorROS
from robocon.m2t.ros2gen import GeneratorROS2


class TestTFBridgeCodeGeneration:
    """Test TFBridge prefix usage in generated code."""
    
    def test_tf_bridge_uses_prefix_ros(self, tmp_path):
        """Test that TFBridge in ROS generates correct broker URIs with prefix."""
        model_content = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[TF] tf_bridge : "robot/tf";
"""
        model_file = tmp_path / "test_tf.rbr"
        model_file.write_text(model_content)
        
        model, _ = build_model(str(model_file))
        output_dir = tmp_path / "output"
        output_dir.mkdir()
        
        GeneratorROS.generate(model, str(output_dir))
        
        bridge_file = output_dir / "TestRobot_bridge.py"
        assert bridge_file.exists()
        
        content = bridge_file.read_text()
        
        # Verify TFBridge is instantiated with the correct prefix
        assert "TFBridge(" in content
        assert "'robot/tf'" in content
        
        # Verify the TFBridge class uses the prefix for all three topics
        assert "f'{broker_prefix}/tf'" in content
        assert "f'{broker_prefix}/tf_static'" in content
        assert "f'{broker_prefix}/tf2_web_republisher'" in content
    
    def test_tf_bridge_uses_prefix_ros2(self, tmp_path):
        """Test that TFBridge in ROS2 generates correct broker URIs with prefix."""
        model_content = """
Robot[ROS2] TestRobot @ "localhost" {
    TOPICS
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[TF] tf_bridge : "robot/transforms";
"""
        model_file = tmp_path / "test_tf.rbr"
        model_file.write_text(model_content)
        
        model, _ = build_model(str(model_file))
        output_dir = tmp_path / "output"
        output_dir.mkdir()
        
        GeneratorROS2.generate(model, str(output_dir))
        
        bridge_file = output_dir / "TestRobot_bridges.py"
        assert bridge_file.exists()
        
        content = bridge_file.read_text()
        
        # Verify TFBridge is instantiated with the correct prefix
        assert "TFBridge(" in content
        assert "'robot/transforms'" in content
        
        # Verify the TFBridge class uses the prefix for all three topics
        assert "f'{broker_prefix}/tf'" in content
        assert "f'{broker_prefix}/tf_static'" in content
        assert "f'{broker_prefix}/tf2_web_republisher'" in content
    
    def test_tf_bridge_default_prefix(self, tmp_path):
        """Test that TFBridge uses bridge name as default prefix when not specified."""
        model_content = """
Robot[ROS2] TestRobot @ "localhost" {
    TOPICS
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[TF] my_tf_bridge;
"""
        model_file = tmp_path / "test_tf_default.rbr"
        model_file.write_text(model_content)
        
        model, _ = build_model(str(model_file))
        output_dir = tmp_path / "output"
        output_dir.mkdir()
        
        GeneratorROS2.generate(model, str(output_dir))
        
        bridge_file = output_dir / "TestRobot_bridges.py"
        content = bridge_file.read_text()
        
        # Verify TFBridge uses the bridge name as default prefix
        assert "'my_tf_bridge'" in content


class TestNodeBridgeCodeGeneration:
    """Test NodeBridge prefix usage in generated code."""
    
    def test_node_bridge_uses_prefix_for_topics(self, tmp_path):
        """Test that NodeBridge generates correct broker URIs with prefix for topics."""
        model_content = """
Robot[ROS2] TestRobot @ "localhost" {
    TOPICS
        pose [ "geometry_msgs/msg/Pose", "/pose" ]
        velocity [ "geometry_msgs/msg/Twist", "/velocity" ]
    NODES
        controller {
            publishes: [ pose ]
            subscribes: [ velocity ]
        }
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[Node] controller_bridge controller:"robot/controller";
"""
        model_file = tmp_path / "test_node.rbr"
        model_file.write_text(model_content)
        
        model, _ = build_model(str(model_file))
        output_dir = tmp_path / "output"
        output_dir.mkdir()
        
        GeneratorROS2.generate(model, str(output_dir))
        
        bridge_file = output_dir / "TestRobot_bridges.py"
        content = bridge_file.read_text()
        
        # Verify that generated bridges use the prefix
        assert "'robot/controller/pose'" in content
        assert "'robot/controller/velocity'" in content
    
    def test_node_bridge_uses_prefix_for_services(self, tmp_path):
        """Test that NodeBridge generates correct broker URIs with prefix for services."""
        model_content = """
Robot[ROS2] TestRobot @ "localhost" {
    TOPICS
    SERVICES
        start [ "std_srvs/srv/Trigger", "/start" ]
        stop [ "std_srvs/srv/Trigger", "/stop" ]
    NODES
        controller {
            services: [ start, stop ]
        }
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[Node] controller_bridge controller:"robot/controller";
"""
        model_file = tmp_path / "test_node_services.rbr"
        model_file.write_text(model_content)
        
        model, _ = build_model(str(model_file))
        output_dir = tmp_path / "output"
        output_dir.mkdir()
        
        GeneratorROS2.generate(model, str(output_dir))
        
        bridge_file = output_dir / "TestRobot_bridges.py"
        content = bridge_file.read_text()
        
        # Verify that generated service bridges use the prefix
        assert "'robot/controller/start'" in content
        assert "'robot/controller/stop'" in content
    
    @pytest.mark.skip(reason="ActionBridge code generation not yet implemented")
    def test_node_bridge_uses_prefix_for_actions(self, tmp_path):
        """Test that NodeBridge generates correct broker URIs with prefix for actions (ROS2 only)."""
        model_content = """
Robot[ROS2] TestRobot @ "localhost" {
    TOPICS
    ACTIONS
        navigate [ "nav2_msgs/action/NavigateToPose", "/navigate_to_pose" ]
    NODES
        controller {
            actions: [ navigate ]
        }
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[Node] controller_bridge controller:"robot/controller";
"""
        model_file = tmp_path / "test_node_actions.rbr"
        model_file.write_text(model_content)
        
        model, _ = build_model(str(model_file))
        output_dir = tmp_path / "output"
        output_dir.mkdir()
        
        GeneratorROS2.generate(model, str(output_dir))
        
        bridge_file = output_dir / "TestRobot_bridges.py"
        content = bridge_file.read_text()
        
        # Verify that generated action bridges use the prefix
        assert "'robot/controller/navigate'" in content


class TestExplicitBridgeMappings:
    """Test that TopicBridge, ServiceBridge, and ActionBridge use explicit mappings."""
    
    def test_topic_bridge_uses_explicit_mapping(self, tmp_path):
        """Test that TopicBridge uses the exact broker URI specified."""
        model_content = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        odom [ "nav_msgs/Odometry", "/odom" ]
        cmd_vel [ "geometry_msgs/Twist", "/cmd_vel" ]
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[Topic] odom_bridge odom -> "sensors/odometry";
Bridge[Topic] cmd_bridge "commands/velocity" -> cmd_vel;
"""
        model_file = tmp_path / "test_topic.rbr"
        model_file.write_text(model_content)
        
        model, _ = build_model(str(model_file))
        output_dir = tmp_path / "output"
        output_dir.mkdir()
        
        GeneratorROS.generate(model, str(output_dir))
        
        bridge_file = output_dir / "TestRobot_bridge.py"
        content = bridge_file.read_text()
        
        # Verify exact broker URIs are used (no prefix modification)
        assert "'sensors/odometry'" in content
        assert "'commands/velocity'" in content
    
    def test_service_bridge_uses_explicit_mapping(self, tmp_path):
        """Test that ServiceBridge uses the exact broker URI specified."""
        model_content = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
    SERVICES
        reset [ "std_srvs/Empty", "/reset" ]
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[Service] reset_bridge "services/system/reset" -> reset;
"""
        model_file = tmp_path / "test_service.rbr"
        model_file.write_text(model_content)
        
        model, _ = build_model(str(model_file))
        output_dir = tmp_path / "output"
        output_dir.mkdir()
        
        GeneratorROS.generate(model, str(output_dir))
        
        bridge_file = output_dir / "TestRobot_bridge.py"
        content = bridge_file.read_text()
        
        # Verify exact broker URI is used
        assert "'services/system/reset'" in content
    
    @pytest.mark.skip(reason="ActionBridge code generation not yet implemented")
    def test_action_bridge_uses_explicit_mapping(self, tmp_path):
        """Test that ActionBridge uses the exact broker URI specified (ROS2 only)."""
        model_content = """
Robot[ROS2] TestRobot @ "localhost" {
    TOPICS
    ACTIONS
        navigate [ "nav2_msgs/action/NavigateToPose", "/navigate_to_pose" ]
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[Action] nav_bridge "actions/navigation/go_to_pose" -> navigate;
"""
        model_file = tmp_path / "test_action.rbr"
        model_file.write_text(model_content)
        
        model, _ = build_model(str(model_file))
        output_dir = tmp_path / "output"
        output_dir.mkdir()
        
        GeneratorROS2.generate(model, str(output_dir))
        
        bridge_file = output_dir / "TestRobot_bridges.py"
        content = bridge_file.read_text()
        
        # Verify exact broker URI is used
        assert "'actions/navigation/go_to_pose'" in content


class TestComprehensiveModel:
    """Test a comprehensive model with all bridge types to ensure correct URI generation."""
    
    def test_comprehensive_ros2_model(self, tmp_path):
        """Test a model with TFBridge, NodeBridge, and explicit bridges."""
        model_content = """
Robot[ROS2] MyRobot @ "localhost" {
    TOPICS
        odom [ "nav_msgs/msg/Odometry", "/odom" ]
        cmd_vel [ "geometry_msgs/msg/Twist", "/cmd_vel" ]
        pose [ "geometry_msgs/msg/Pose", "/pose" ]
        status [ "std_msgs/msg/String", "/status" ]
    
    SERVICES
        reset [ "std_srvs/srv/Empty", "/reset" ]
        start [ "std_srvs/srv/Trigger", "/start" ]
    
    ACTIONS
        navigate [ "nav2_msgs/action/NavigateToPose", "/navigate" ]
    
    NODES
        base_controller {
            publishes: [ pose, status ]
            subscribes: [ cmd_vel ]
            services: [ start ]
            actions: [ navigate ]
        }
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

// Explicit topic bridge with custom mapping
Bridge[Topic] odom_bridge odom -> "sensors/robot/odometry";

// Explicit service bridge with custom mapping
Bridge[Service] reset_bridge "services/robot/reset" -> reset;

// NodeBridge with prefix
Bridge[Node] base_bridges base_controller:"robot/base";

// TFBridge with prefix
Bridge[TF] tf_bridge : "robot/transforms";
"""
        model_file = tmp_path / "test_comprehensive.rbr"
        model_file.write_text(model_content)
        
        model, _ = build_model(str(model_file))
        output_dir = tmp_path / "output"
        output_dir.mkdir()
        
        GeneratorROS2.generate(model, str(output_dir))
        
        bridge_file = output_dir / "MyRobot_bridges.py"
        assert bridge_file.exists()
        
        content = bridge_file.read_text()
        
        # Verify explicit TopicBridge uses exact mapping
        assert "'sensors/robot/odometry'" in content
        
        # Verify explicit ServiceBridge uses exact mapping
        assert "'services/robot/reset'" in content
        
        # Verify NodeBridge uses prefix for all expanded bridges
        assert "'robot/base/pose'" in content
        assert "'robot/base/status'" in content
        assert "'robot/base/cmd_vel'" in content
        assert "'robot/base/start'" in content
        # Note: ActionBridge code generation not yet implemented, so these are commented out
        # assert "'robot/base/navigate'" in content
        
        # Verify TFBridge uses prefix
        assert "'robot/transforms'" in content
        
        # Verify TFBridge class template uses prefix correctly
        assert "f'{broker_prefix}/tf'" in content
        assert "f'{broker_prefix}/tf_static'" in content
        assert "f'{broker_prefix}/tf2_web_republisher'" in content
    
    def test_comprehensive_ros_model(self, tmp_path):
        """Test a comprehensive ROS model (no actions)."""
        model_content = """
Robot[ROS] MyRobot @ "localhost" {
    TOPICS
        odom [ "nav_msgs/Odometry", "/odom" ]
        cmd_vel [ "geometry_msgs/Twist", "/cmd_vel" ]
        pose [ "geometry_msgs/Pose", "/pose" ]
    
    SERVICES
        reset [ "std_srvs/Empty", "/reset" ]
    
    NODES
        controller {
            publishes: [ pose ]
            subscribes: [ cmd_vel ]
            services: [ reset ]
        }
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[Topic] odom_bridge odom -> "data/odom";
Bridge[Node] ctrl_bridges controller:"robot/ctrl";
Bridge[TF] tf_bridge : "robot/tf";
"""
        model_file = tmp_path / "test_ros_comprehensive.rbr"
        model_file.write_text(model_content)
        
        model, _ = build_model(str(model_file))
        output_dir = tmp_path / "output"
        output_dir.mkdir()
        
        GeneratorROS.generate(model, str(output_dir))
        
        bridge_file = output_dir / "MyRobot_bridge.py"
        content = bridge_file.read_text()
        
        # Verify explicit mapping
        assert "'data/odom'" in content
        
        # Verify NodeBridge prefix
        assert "'robot/ctrl/pose'" in content
        assert "'robot/ctrl/cmd_vel'" in content
        assert "'robot/ctrl/reset'" in content
        
        # Verify TFBridge prefix
        assert "'robot/tf'" in content
