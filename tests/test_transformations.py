import os
import pytest
from robocon.utils import build_model, get_mm
from robocon.definitions import TMP_DIR

@pytest.fixture
def transformation_model_content():
    return """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        t_sensor ["std_msgs/Float32", "/sensor"]
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[Topic] sensor_bridge t_sensor : "iot/telemetry" {
    transform: {
        temp_f: "msg.data * 1.8 + 32",
        status: "'CRITICAL' if msg.data > 100 else 'OK'"
    }
};
"""

def test_transformation_grammar_parsing(transformation_model_content, tmp_path):
    model_file = tmp_path / "test_transform.rbr"
    model_file.write_text(transformation_model_content)
    
    model, _ = build_model(str(model_file))
    
    assert len(model.bridges) == 1
    bridge = model.bridges[0]
    assert bridge.name == "sensor_bridge"
    assert hasattr(bridge, 'transforms')
    assert len(bridge.transforms) == 2
    
    transforms = {t.target: t.expression for t in bridge.transforms}
    assert "temp_f" in transforms
    assert transforms["temp_f"] == "msg.data * 1.8 + 32"
    assert "status" in transforms
    assert transforms["status"] == "'CRITICAL' if msg.data > 100 else 'OK'"

def test_bridge_processor_transform_dict(transformation_model_content, tmp_path):
    model_file = tmp_path / "test_transform_processor.rbr"
    model_file.write_text(transformation_model_content)
    
    model, _ = build_model(str(model_file))
    bridge = model.bridges[0]
    
    assert hasattr(bridge, 'has_transform')
    assert bridge.has_transform is True
    assert hasattr(bridge, 'transform_dict')
    assert bridge.transform_dict["temp_f"] == "msg.data * 1.8 + 32"

def test_node_bridge_expansion_with_transform_compatibility(tmp_path):
    # Ensure NodeBridge expansion still works and doesn't crash with new grammar
    model_content = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        t1 ["std_msgs/String", "/t1"]
    NODES
        n1 {
            publishes: [t1]
        }
}
Broker[MQTT] TestBroker { host: "localhost", port: 1883 }
Bridge[Node] nb n1;
"""
    model_file = tmp_path / "test_node_transform.rbr"
    model_file.write_text(model_content)
    
    model, _ = build_model(str(model_file))
    # NodeBridge should be expanded to TopicBridge
    assert len(model.bridges) == 1
    assert model.bridges[0].__class__.__name__ == "TopicBridge"
    assert model.bridges[0].has_transform is False

def test_generated_code_contains_dotdict(transformation_model_content, tmp_path):
    """Verify that generated ROS bridge code contains DotDict helper."""
    from robocon.m2t.rosgen import GeneratorROS
    
    model_file = tmp_path / "test_dotdict.rbr"
    model_file.write_text(transformation_model_content)
    
    model, _ = build_model(str(model_file))
    output_dir = tmp_path / "output"
    output_dir.mkdir()
    
    GeneratorROS.generate(model, str(output_dir))
    
    # Check that DotDict is in the generated code
    bridge_file = output_dir / "TestRobot_bridge.py"
    assert bridge_file.exists()
    
    content = bridge_file.read_text()
    assert "class DotDict(dict):" in content
    assert "def __getattr__(self, name):" in content

def test_generated_code_uses_transform_dict(transformation_model_content, tmp_path):
    """Verify that generated code passes transform_dict to bridge classes."""
    from robocon.m2t.rosgen import GeneratorROS
    
    model_file = tmp_path / "test_transform_usage.rbr"
    model_file.write_text(transformation_model_content)
    
    model, _ = build_model(str(model_file))
    output_dir = tmp_path / "output"
    output_dir.mkdir()
    
    GeneratorROS.generate(model, str(output_dir))
    
    bridge_file = output_dir / "TestRobot_bridge.py"
    content = bridge_file.read_text()
    
    # Verify transform_dict is passed to bridge initialization
    assert "transform=" in content
    assert "temp_f" in content
    assert "msg.data * 1.8 + 32" in content

def test_transformation_with_nested_fields(tmp_path):
    """Test transformations with nested field access using dot notation."""
    model_content = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        t_data ["std_msgs/String", "/data"]
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[Topic] data_bridge t_data : "iot/data" {
    transform: {
        value: "msg.nested.field * 2",
        flag: "msg.status == 'active'"
    }
};
"""
    model_file = tmp_path / "test_nested.rbr"
    model_file.write_text(model_content)
    
    model, _ = build_model(str(model_file))
    bridge = model.bridges[0]
    
    assert bridge.has_transform is True
    assert "msg.nested.field * 2" in bridge.transform_dict.values()
    assert "msg.status == 'active'" in bridge.transform_dict.values()
