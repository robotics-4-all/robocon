"""Tests for NodeBridge URI validation and code generation."""
import pytest
import os
from pathlib import Path
from robocon.utils import get_mm, build_model
from robocon.m2t.rosgen import GeneratorROS
from robocon.m2t.ros2gen import GeneratorROS2


def test_nodebridge_ros_uris_with_prefix(tmp_path):
    """Test that NodeBridge correctly sets ROS topic URIs from node definition."""
    model_path = tmp_path / "test.rbr"
    model_content = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        pose [ "geometry_msgs/Pose", "/robot/pose" ]
        cmd_vel [ "geometry_msgs/Twist", "/robot/cmd_vel" ]
    
    NODES
        controller {
            publishes: [pose]
            subscribes: [cmd_vel]
        }
}

Broker[MQTT] mqtt {
    host: "localhost",
    port: 1883
}

Bridge[Node] ctrl_bridge controller:"myrobot";
    """
    model_path.write_text(model_content)
    
    mm = get_mm()
    model, _ = build_model(str(model_path))
    
    # Should have 2 topic bridges
    assert len(model.bridges) == 2
    
    # Check ROS URIs match the topic definitions
    pose_bridge = [b for b in model.bridges if b.topic.name == 'pose'][0]
    assert pose_bridge.ros_endpoint.uri == '/robot/pose'
    assert pose_bridge.topic.uri == '/robot/pose'
    
    cmd_bridge = [b for b in model.bridges if b.topic.name == 'cmd_vel'][0]
    assert cmd_bridge.ros_endpoint.uri == '/robot/cmd_vel'
    assert cmd_bridge.topic.uri == '/robot/cmd_vel'


def test_nodebridge_broker_uris_with_prefix(tmp_path):
    """Test that NodeBridge correctly builds broker URIs with prefix."""
    model_path = tmp_path / "test.rbr"
    model_content = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        pose [ "geometry_msgs/Pose", "/robot/pose" ]
        velocity [ "geometry_msgs/Twist", "/robot/velocity" ]
    
    NODES
        controller {
            publishes: [pose]
            subscribes: [velocity]
        }
}

Broker[MQTT] mqtt {
    host: "localhost",
    port: 1883
}

Bridge[Node] ctrl_bridge controller:"robot/main";
    """
    model_path.write_text(model_content)
    
    mm = get_mm()
    model, _ = build_model(str(model_path))
    
    # Check broker URIs use the prefix
    pose_bridge = [b for b in model.bridges if b.topic.name == 'pose'][0]
    assert pose_bridge.brokerURI == 'robot/main/pose'
    
    vel_bridge = [b for b in model.bridges if b.topic.name == 'velocity'][0]
    assert vel_bridge.brokerURI == 'robot/main/velocity'


def test_nodebridge_broker_uris_without_prefix(tmp_path):
    """Test that NodeBridge uses topic names when no prefix is provided."""
    model_path = tmp_path / "test.rbr"
    model_content = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        status [ "std_msgs/String", "/status" ]
        diagnostics [ "diagnostic_msgs/DiagnosticArray", "/diagnostics" ]
    
    NODES
        sensor {
            publishes: [status, diagnostics]
        }
}

Broker[MQTT] mqtt {
    host: "localhost",
    port: 1883
}

Bridge[Node] sensor_bridge sensor;
    """
    model_path.write_text(model_content)
    
    mm = get_mm()
    model, _ = build_model(str(model_path))
    
    # Check broker URIs are just the topic names
    status_bridge = [b for b in model.bridges if b.topic.name == 'status'][0]
    assert status_bridge.brokerURI == 'status'
    
    diag_bridge = [b for b in model.bridges if b.topic.name == 'diagnostics'][0]
    assert diag_bridge.brokerURI == 'diagnostics'


def test_nodebridge_broker_uris_with_custom_mappings(tmp_path):
    """Test that custom mappings override default broker URIs."""
    model_path = tmp_path / "test.rbr"
    model_content = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        pose [ "geometry_msgs/Pose", "/robot/pose" ]
        velocity [ "geometry_msgs/Twist", "/robot/velocity" ]
        status [ "std_msgs/String", "/robot/status" ]
    
    NODES
        controller {
            publishes: [pose, status]
            subscribes: [velocity]
        }
}

Broker[MQTT] mqtt {
    host: "localhost",
    port: 1883
}

Bridge[Node] ctrl_bridge controller:"robot" {
    topic_maps: {
        pose: "telemetry/position",
        velocity: "commands/vel"
    }
};
    """
    model_path.write_text(model_content)
    
    mm = get_mm()
    model, _ = build_model(str(model_path))
    
    # Check custom mapped topics use custom URIs
    pose_bridge = [b for b in model.bridges if b.topic.name == 'pose'][0]
    assert pose_bridge.brokerURI == 'telemetry/position'
    
    vel_bridge = [b for b in model.bridges if b.topic.name == 'velocity'][0]
    assert vel_bridge.brokerURI == 'commands/vel'
    
    # Check unmapped topic uses prefix
    status_bridge = [b for b in model.bridges if b.topic.name == 'status'][0]
    assert status_bridge.brokerURI == 'robot/status'


def test_nodebridge_service_uris(tmp_path):
    """Test that NodeBridge correctly sets service URIs."""
    model_path = tmp_path / "test.rbr"
    model_content = """
Robot[ROS] TestRobot @ "localhost" {
    SERVICES
        reset [ "std_srvs/Empty", "/robot/reset" ]
        calibrate [ "std_srvs/Trigger", "/robot/calibrate" ]
    
    NODES
        controller {
            services: [reset, calibrate]
        }
}

Broker[MQTT] mqtt {
    host: "localhost",
    port: 1883
}

Bridge[Node] ctrl_bridge controller:"robot/ctrl" {
    service_maps: {
        reset: "services/emergency_reset"
    }
};
    """
    model_path.write_text(model_content)
    
    mm = get_mm()
    model, _ = build_model(str(model_path))
    
    # Check ROS URIs
    reset_bridge = [b for b in model.bridges if b.service.name == 'reset'][0]
    assert reset_bridge.ros_endpoint.uri == '/robot/reset'
    assert reset_bridge.service.uri == '/robot/reset'
    
    cal_bridge = [b for b in model.bridges if b.service.name == 'calibrate'][0]
    assert reset_bridge.ros_endpoint.uri == '/robot/reset'
    
    # Check broker URIs
    assert reset_bridge.brokerURI == 'services/emergency_reset'  # Custom
    assert cal_bridge.brokerURI == 'robot/ctrl/calibrate'  # Prefix


def test_generated_code_contains_correct_uris_ros(tmp_path):
    """Test that generated ROS code contains the correct broker URIs."""
    model_path = tmp_path / "test.rbr"
    model_content = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        pose [ "geometry_msgs/Pose", "/robot/pose" ]
        cmd_vel [ "geometry_msgs/Twist", "/cmd_vel" ]
    
    NODES
        controller {
            publishes: [pose]
            subscribes: [cmd_vel]
        }
}

Broker[MQTT] mqtt {
    host: "localhost",
    port: 1883
}

Bridge[Node] ctrl_bridge controller:"robot" {
    topic_maps: {
        pose: "telemetry/robot/position"
    }
};
    """
    model_path.write_text(model_content)
    
    mm = get_mm()
    model, _ = build_model(str(model_path))
    
    # Generate code
    output_dir = tmp_path / "generated"
    gen = GeneratorROS()
    gen.generate(model, output_dir)
    
    # Check that bridge file exists (ROS generates one consolidated file)
    bridge_files = list(output_dir.glob("*_bridge.py"))
    assert len(bridge_files) == 1
    
    # Read all generated content
    content = bridge_files[0].read_text()
    
    # Verify pose bridge has custom URI
    assert 'telemetry/robot/position' in content, "Should contain custom URI"
    pose_content = content
    assert 'telemetry/robot/position' in pose_content
    assert '/robot/pose' in pose_content  # ROS topic
    
    # Verify cmd_vel bridge has prefix-based URI
    assert 'robot/cmd_vel' in content
    assert '/cmd_vel' in content  # ROS topic


def test_generated_code_contains_correct_uris_ros2(tmp_path):
    """Test that generated ROS2 code contains the correct broker URIs."""
    model_path = tmp_path / "test.rbr"
    model_content = """
Robot[ROS2] TestRobot @ "localhost" {
    TOPICS
        odom [ "nav_msgs/msg/Odometry", "/odom" ]
        cmd_vel [ "geometry_msgs/msg/Twist", "/cmd_vel" ]
    
    SERVICES
        reset [ "std_srvs/srv/Empty", "/reset" ]
    
    NODES
        controller {
            publishes: [odom]
            subscribes: [cmd_vel]
            services: [reset]
        }
}

Broker[MQTT] mqtt {
    host: "localhost",
    port: 1883
}

Bridge[Node] ctrl_bridge controller {
    topic_maps: {
        odom: "telemetry/odometry",
        cmd_vel: "commands/velocity"
    }
    service_maps: {
        reset: "services/reset"
    }
};
    """
    model_path.write_text(model_content)
    
    mm = get_mm()
    model, _ = build_model(str(model_path))
    
    # Generate code
    output_dir = tmp_path / "generated"
    gen = GeneratorROS2()
    gen.generate(model, output_dir)
    
    # Check that bridge files exist (ROS2 generates consolidated bridge file)
    bridge_files = list(output_dir.glob("*_bridges.py"))  # Note: _bridges not _bridge
    if not bridge_files:
        # Fallback to old pattern
        bridge_files = list(output_dir.glob("*_bridge.py"))
    assert len(bridge_files) >= 1, f"No bridge files found in {output_dir}"
    
    # Verify URIs are in generated code
    all_content = ""
    for f in bridge_files:
        all_content += f.read_text()
    
    # Check custom topic mappings
    assert 'telemetry/odometry' in all_content
    assert 'commands/velocity' in all_content
    assert 'services/reset' in all_content
    
    # Check ROS URIs
    assert '/odom' in all_content
    assert '/cmd_vel' in all_content
    assert '/reset' in all_content


def test_generated_code_no_prefix(tmp_path):
    """Test that generated code correctly handles no prefix case."""
    model_path = tmp_path / "test.rbr"
    model_content = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        status [ "std_msgs/String", "/status" ]
    
    NODES
        sensor {
            publishes: [status]
        }
}

Broker[MQTT] mqtt {
    host: "localhost",
    port: 1883
}

Bridge[Node] sensor_bridge sensor;
    """
    model_path.write_text(model_content)
    
    mm = get_mm()
    model, _ = build_model(str(model_path))
    
    # Generate code
    output_dir = tmp_path / "generated"
    gen = GeneratorROS()
    gen.generate(model, output_dir)
    
    # Check generated file
    bridge_files = list(output_dir.glob("*_bridge.py"))
    assert len(bridge_files) >= 1
    
    content = bridge_files[0].read_text()
    # Should have topic name as broker URI (no prefix) - look for the string in generated code
    assert '"status"' in content or "'status'" in content
    assert '/status' in content  # ROS URI


def test_generated_code_broker_config(tmp_path):
    """Test that generated code contains correct broker configuration."""
    model_path = tmp_path / "test.rbr"
    model_content = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        pose [ "geometry_msgs/Pose", "/pose" ]
    
    NODES
        controller {
            publishes: [pose]
        }
}

Broker[MQTT] mqtt {
    host: "mqtt.example.com",
    port: 8883,
    auth.username: "testuser",
    auth.password: "testpass"
}

Bridge[Node] ctrl_bridge controller:"robot";
    """
    model_path.write_text(model_content)
    
    mm = get_mm()
    model, _ = build_model(str(model_path))
    
    # Generate code
    output_dir = tmp_path / "generated"
    gen = GeneratorROS()
    gen.generate(model, output_dir)
    
    # Check bridge file
    bridge_files = list(output_dir.glob("*_bridge.py"))
    content = bridge_files[0].read_text()
    
    # Verify broker configuration
    assert 'mqtt.example.com' in content
    assert '8883' in content
    assert 'testuser' in content
    assert 'testpass' in content


def test_end_to_end_complex_nodebridge(tmp_path):
    """End-to-end test with complex NodeBridge configuration."""
    model_path = tmp_path / "test.rbr"
    model_content = """
Robot[ROS2] ComplexRobot @ "localhost" {
    TOPICS
        odom [ "nav_msgs/msg/Odometry", "/odom" ]
        cmd_vel [ "geometry_msgs/msg/Twist", "/cmd_vel" ]
        status [ "std_msgs/msg/String", "/status" ]
        lidar [ "sensor_msgs/msg/LaserScan", "/scan" ]
    
    SERVICES
        reset [ "std_srvs/srv/Empty", "/reset" ]
        calibrate [ "std_srvs/srv/Trigger", "/calibrate" ]
    
    ACTIONS
        navigate [ "nav2_msgs/action/NavigateToPose", "/navigate" ]
    
    NODES
        main_controller {
            publishes: [odom, status]
            subscribes: [cmd_vel]
            services: [reset, calibrate]
            actions: [navigate]
        }
        
        lidar_node {
            publishes: [lidar]
        }
}

Broker[MQTT] mqtt {
    host: "localhost",
    port: 1883
}

Bridge[Node] main_bridge main_controller:"robot/main" {
    topic_maps: {
        odom: "telemetry/odometry",
        cmd_vel: "commands/velocity"
    }
    service_maps: {
        reset: "services/emergency_reset"
    }
    action_maps: {
        navigate: "actions/navigation/goal"
    }
};

Bridge[Node] lidar_bridge lidar_node;
    """
    model_path.write_text(model_content)
    
    mm = get_mm()
    model, _ = build_model(str(model_path))
    
    # Verify model processing
    # main_bridge should create 5 bridges: 2 topics + 1 topic + 2 services + 1 action
    # lidar_bridge should create 1 bridge: 1 topic
    main_bridges = [b for b in model.bridges if b.name.startswith('main_bridge_')]
    lidar_bridges = [b for b in model.bridges if b.name.startswith('lidar_bridge_')]
    
    assert len(main_bridges) == 6  # odom, status, cmd_vel, reset, calibrate, navigate
    assert len(lidar_bridges) == 1  # lidar
    
    # Verify URIs on main_bridge
    odom_b = [b for b in main_bridges if hasattr(b, 'topic') and b.topic.name == 'odom'][0]
    assert odom_b.brokerURI == 'telemetry/odometry'
    
    status_b = [b for b in main_bridges if hasattr(b, 'topic') and b.topic.name == 'status'][0]
    assert status_b.brokerURI == 'robot/main/status'  # Uses prefix
    
    cmd_b = [b for b in main_bridges if hasattr(b, 'topic') and b.topic.name == 'cmd_vel'][0]
    assert cmd_b.brokerURI == 'commands/velocity'
    
    reset_b = [b for b in main_bridges if hasattr(b, 'service') and b.service.name == 'reset'][0]
    assert reset_b.brokerURI == 'services/emergency_reset'
    
    cal_b = [b for b in main_bridges if hasattr(b, 'service') and b.service.name == 'calibrate'][0]
    assert cal_b.brokerURI == 'robot/main/calibrate'  # Uses prefix
    
    nav_b = [b for b in main_bridges if hasattr(b, 'action')][0]
    assert nav_b.brokerURI == 'actions/navigation/goal'
    
    # Verify lidar bridge (no prefix)
    lidar_b = lidar_bridges[0]
    assert lidar_b.brokerURI == 'lidar'
    
    # Generate code
    output_dir = tmp_path / "generated"
    gen = GeneratorROS2()
    gen.generate(model, output_dir)
    
    # Verify all URIs are in generated code
    bridge_files = list(output_dir.glob("*_bridges.py"))  # Note: _bridges not _bridge
    if not bridge_files:
        bridge_files = list(output_dir.glob("*_bridge.py"))
    
    assert len(bridge_files) >= 1, f"No bridge files found in {output_dir}"
    
    all_content = ""
    for f in bridge_files:
        all_content += f.read_text()
    
    # Custom mappings (topics and services only - actions not yet implemented in generators)
    assert 'telemetry/odometry' in all_content
    assert 'commands/velocity' in all_content
    assert 'services/emergency_reset' in all_content
    # Note: actions like 'actions/navigation/goal' are not generated yet
    
    # Prefix-based
    assert 'robot/main/status' in all_content
    assert 'robot/main/calibrate' in all_content
    
    # No prefix
    assert '"lidar"' in all_content or "'lidar'" in all_content
    
    # ROS URIs (for topics and services - actions not yet implemented)
    assert '/odom' in all_content
    assert '/cmd_vel' in all_content
    assert '/status' in all_content
    assert '/scan' in all_content
    assert '/reset' in all_content
    assert '/calibrate' in all_content
    # Note: /navigate (action) not generated yet
