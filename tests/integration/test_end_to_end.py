"""
End-to-end workflow tests.
"""
import pytest
import os
from robocon.utils import build_model, get_mm
from robocon.m2t.rosgen import GeneratorROS
from robocon.m2t.ros2gen import GeneratorROS2


class TestRosWorkflow:
    """End-to-end tests for ROS workflow."""
    
    def test_complete_ros_workflow(self, temp_dir, sample_ros_model):
        """Test complete ROS workflow: parse → validate → generate → verify."""
        # Step 1: Parse the model
        mm = get_mm()
        model = mm.model_from_str(sample_ros_model)
        assert model is not None
        
        # Step 2: Validate model structure
        assert model.robot.type == "ROS"
        assert model.robot.name == "TestRobot"
        assert len(model.robot.topics) == 2
        assert len(model.robot.services) == 1
        assert len(model.bridges) == 3
        
        # Step 3: Generate code
        out_dir = os.path.join(temp_dir, "ros_output")
        GeneratorROS.generate(model, out_dir)
        
        # Step 4: Verify output
        bridge_file = os.path.join(out_dir, f"{model.robot.name}_bridge.py")
        req_file = os.path.join(out_dir, "requirements.txt")
        
        assert os.path.exists(bridge_file)
        assert os.path.exists(req_file)
        
        # Verify generated code is valid Python
        with open(bridge_file, 'r') as f:
            code = f.read()
            compile(code, bridge_file, 'exec')


class TestRos2Workflow:
    """End-to-end tests for ROS2 workflow."""
    
    def test_complete_ros2_workflow(self, temp_dir, sample_ros2_model):
        """Test complete ROS2 workflow: parse → validate → generate → verify."""
        # Step 1: Parse the model
        mm = get_mm()
        model = mm.model_from_str(sample_ros2_model)
        assert model is not None
        
        # Step 2: Validate model structure
        assert model.robot.type == "ROS2"
        assert model.robot.name == "TestRobot2"
        assert len(model.robot.topics) == 2
        assert len(model.robot.services) == 1
        assert len(model.robot.actions) == 1
        assert len(model.bridges) == 2
        
        # Step 3: Generate code
        out_dir = os.path.join(temp_dir, "ros2_output")
        GeneratorROS2.generate(model, out_dir)
        
        # Step 4: Verify output
        bridge_file = os.path.join(out_dir, f"{model.robot.name}_bridges.py")
        req_file = os.path.join(out_dir, "requirements.txt")
        
        assert os.path.exists(bridge_file)
        assert os.path.exists(req_file)
        
        # Verify generated code is valid Python
        with open(bridge_file, 'r') as f:
            code = f.read()
            compile(code, bridge_file, 'exec')


class TestNodeBridgeExpansion:
    """Tests for NodeBridge expansion workflow."""
    
    def test_node_bridge_expansion_workflow(self, temp_dir, sample_node_bridge_model):
        """Test NodeBridge expansion produces correct bridges."""
        # Parse model
        mm = get_mm()
        model = mm.model_from_str(sample_node_bridge_model)
        
        # Verify NodeBridge was expanded
        assert len(model.bridges) > 1
        
        # Count bridge types
        topic_bridges = [b for b in model.bridges if b.__class__.__name__ == 'TopicBridge']
        service_bridges = [b for b in model.bridges if b.__class__.__name__ == 'ServiceBridge']
        
        assert len(topic_bridges) == 3  # pose, velocity, status
        assert len(service_bridges) == 2  # start, stop
        
        # Verify directions
        r2b_count = len([b for b in model.bridges if b.direction == 'R2B'])
        b2r_count = len([b for b in model.bridges if b.direction == 'B2R'])
        
        assert r2b_count == 2  # pose and status (published)
        assert b2r_count == 3  # velocity (subscribed) + start + stop (services)
        
        # Verify prefix in broker URIs
        for bridge in model.bridges:
            assert bridge.brokerURI.startswith("robot/controller/")
        
        # Generate code
        out_dir = os.path.join(temp_dir, "node_bridge_output")
        GeneratorROS2.generate(model, out_dir)
        
        # Verify output
        bridge_file = os.path.join(out_dir, f"{model.robot.name}_bridges.py")
        assert os.path.exists(bridge_file)


class TestMultiBridgeModel:
    """Tests for models with multiple bridge types."""
    
    def test_multi_bridge_generation(self, temp_dir):
        """Test generation with multiple bridge types."""
        multi_bridge_model = """
Robot[ROS2] MultiBridge @ "localhost" {
    TOPICS
        sensor_data [ "sensor_msgs/msg/LaserScan", "/scan" ]
        robot_state [ "std_msgs/msg/String", "/state" ]
    
    SERVICES
        trigger [ "std_srvs/srv/Trigger", "/trigger" ]
    
    ACTIONS
        move [ "nav2_msgs/action/NavigateToPose", "/navigate" ]
}

Broker[MQTT] TestBroker {
    host: "broker.local",
    port: 1883
}

Bridge[Topic] sensor_bridge sensor_data -> "sensors/scan";
Bridge[Topic] state_bridge robot_state -> "status/state";
Bridge[Service] trigger_bridge "commands/trigger" -> trigger;
Bridge[Action] move_bridge "commands/move" -> move;
"""
        
        mm = get_mm()
        model = mm.model_from_str(multi_bridge_model)
        
        # Verify all bridge types
        assert len(model.bridges) == 4
        
        topic_bridges = [b for b in model.bridges if b.__class__.__name__ == 'TopicBridge']
        service_bridges = [b for b in model.bridges if b.__class__.__name__ == 'ServiceBridge']
        action_bridges = [b for b in model.bridges if b.__class__.__name__ == 'ActionBridge']
        
        assert len(topic_bridges) == 2
        assert len(service_bridges) == 1
        assert len(action_bridges) == 1
        
        # Generate and verify
        out_dir = os.path.join(temp_dir, "multi_bridge_output")
        GeneratorROS2.generate(model, out_dir)
        
        bridge_file = os.path.join(out_dir, f"{model.robot.name}_bridges.py")
        assert os.path.exists(bridge_file)
        
        # Verify code syntax
        with open(bridge_file, 'r') as f:
            code = f.read()
            compile(code, bridge_file, 'exec')
