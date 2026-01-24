"""Tests for enhanced NodeBridge functionality."""
import pytest
from robocon.utils import get_mm, build_model


def test_nodebridge_backward_compatibility(tmp_path):
    """Test that old NodeBridge syntax still works."""
    model_path = tmp_path / "test.rbr"
    model_content = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        pose [ "turtlesim/Pose", "/turtle1/pose" ]
        cmd_vel [ "geometry_msgs/Twist", "/turtle1/cmd_vel" ]
    
    NODES
        turtlesim_node {
            publishes: [pose]
            subscribes: [cmd_vel]
        }
}

Broker[MQTT] mqtt {
    host: "localhost",
    port: 1883
}

Bridge[Node] turtle_bridges turtlesim_node:"turtle1";
    """
    model_path.write_text(model_content)
    
    mm = get_mm()
    model, _ = build_model(str(model_path))
    
    # Should have 2 topic bridges
    assert len(model.bridges) == 2
    assert all(b.__class__.__name__ == 'TopicBridge' for b in model.bridges)
    
    # Check pose bridge (R2B)
    pose_bridge = [b for b in model.bridges if b.topic.name == 'pose'][0]
    assert pose_bridge.direction == 'R2B'
    assert pose_bridge.brokerURI == 'turtle1/pose'
    
    # Check cmd_vel bridge (B2R)
    cmd_vel_bridge = [b for b in model.bridges if b.topic.name == 'cmd_vel'][0]
    assert cmd_vel_bridge.direction == 'B2R'
    assert cmd_vel_bridge.brokerURI == 'turtle1/cmd_vel'


def test_nodebridge_no_prefix(tmp_path):
    """Test NodeBridge without prefix."""
    model_path = tmp_path / "test.rbr"
    model_content = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        topic1 [ "std_msgs/String", "/topic1" ]
    
    NODES
        mynode {
            publishes: [topic1]
        }
}

Broker[MQTT] mqtt {
    host: "localhost",
    port: 1883
}

Bridge[Node] my_bridge mynode;
    """
    model_path.write_text(model_content)
    
    mm = get_mm()
    model, _ = build_model(str(model_path))
    
    # Should have 1 topic bridge
    assert len(model.bridges) == 1
    bridge = model.bridges[0]
    assert bridge.brokerURI == 'topic1'  # No prefix


def test_nodebridge_with_topic_maps(tmp_path):
    """Test NodeBridge with custom topic mappings."""
    model_path = tmp_path / "test.rbr"
    model_content = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        pose [ "turtlesim/Pose", "/turtle1/pose" ]
        color_sensor [ "turtlesim/Color", "/turtle1/color_sensor" ]
        cmd_vel [ "geometry_msgs/Twist", "/turtle1/cmd_vel" ]
    
    NODES
        turtlesim_node {
            publishes: [pose, color_sensor]
            subscribes: [cmd_vel]
        }
}

Broker[MQTT] mqtt {
    host: "localhost",
    port: 1883
}

Bridge[Node] turtle_bridges turtlesim_node:"turtle1" {
    TOPICS
        pose -> "robot/turtle/position",
        cmd_vel -> "robot/turtle/commands/velocity"
};
    """
    model_path.write_text(model_content)
    
    mm = get_mm()
    model, _ = build_model(str(model_path))
    
    # Should have 3 topic bridges
    assert len(model.bridges) == 3
    
    # Check pose bridge (custom mapping)
    pose_bridge = [b for b in model.bridges if b.topic.name == 'pose'][0]
    assert pose_bridge.brokerURI == 'robot/turtle/position'
    
    # Check cmd_vel bridge (custom mapping)
    cmd_vel_bridge = [b for b in model.bridges if b.topic.name == 'cmd_vel'][0]
    assert cmd_vel_bridge.brokerURI == 'robot/turtle/commands/velocity'
    
    # Check color_sensor bridge (uses prefix)
    color_bridge = [b for b in model.bridges if b.topic.name == 'color_sensor'][0]
    assert color_bridge.brokerURI == 'turtle1/color_sensor'


def test_nodebridge_without_prefix_but_with_mappings(tmp_path):
    """Test NodeBridge without prefix but with custom topic mappings."""
    model_path = tmp_path / "test.rbr"
    model_content = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        topic1 [ "std_msgs/String", "/topic1" ]
        topic2 [ "std_msgs/String", "/topic2" ]
    
    NODES
        mynode {
            publishes: [topic1, topic2]
        }
}

Broker[MQTT] mqtt {
    host: "localhost",
    port: 1883
}

Bridge[Node] my_bridge mynode {
    TOPICS
        topic1 -> "custom/topic1"
};
    """
    model_path.write_text(model_content)
    
    mm = get_mm()
    model, _ = build_model(str(model_path))
    
    # Should have 2 topic bridges
    assert len(model.bridges) == 2
    
    # topic1 uses custom mapping
    t1_bridge = [b for b in model.bridges if b.topic.name == 'topic1'][0]
    assert t1_bridge.brokerURI == 'custom/topic1'
    
    # topic2 has no prefix and no mapping, so just the name
    t2_bridge = [b for b in model.bridges if b.topic.name == 'topic2'][0]
    assert t2_bridge.brokerURI == 'topic2'
