import pytest
from robocon.utils import build_model
from textx.exceptions import TextXSemanticError


@pytest.fixture
def when_model_content():
    return """
Robot[ROS] FilterBot @ "localhost" {
    TOPICS
        diagnostics ["std_msgs/String", "/diagnostics"]
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[Topic] critical_only diagnostics -> "alerts/critical" {
    when: "msg.data == 'ERROR' or msg.data == 'FATAL'"
};
"""


@pytest.fixture
def unless_model_content():
    return """
Robot[ROS] FilterBot @ "localhost" {
    TOPICS
        sensor_data ["std_msgs/Float32", "/sensor"]
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[Topic] no_test_data sensor_data -> "iot/sensors" {
    unless: "msg.data < 0"
};
"""


def test_when_clause_parsing(when_model_content, tmp_path):
    """Test that 'when' clauses are parsed correctly."""
    model_file = tmp_path / "test_when.rbr"
    model_file.write_text(when_model_content)
    
    model, _ = build_model(str(model_file))
    
    assert len(model.bridges) == 1
    bridge = model.bridges[0]
    assert bridge.has_condition is True
    assert bridge.condition_type == 'when'
    assert bridge.condition == "msg.data == 'ERROR' or msg.data == 'FATAL'"


def test_unless_clause_parsing(unless_model_content, tmp_path):
    """Test that 'unless' clauses are parsed correctly."""
    model_file = tmp_path / "test_unless.rbr"
    model_file.write_text(unless_model_content)
    
    model, _ = build_model(str(model_file))
    
    assert len(model.bridges) == 1
    bridge = model.bridges[0]
    assert bridge.has_condition is True
    assert bridge.condition_type == 'unless'
    assert bridge.condition == "msg.data < 0"


def test_mutual_exclusivity_validation(tmp_path):
    """Test that having both 'when' and 'unless' raises an error."""
    model_content = """
Robot[ROS] FilterBot @ "localhost" {
    TOPICS
        data ["std_msgs/String", "/data"]
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[Topic] bad_bridge data -> "topic" {
    when: "msg.data == 'test'"
    unless: "msg.data == 'debug'"
};
"""
    model_file = tmp_path / "test_mutual_exclusivity.rbr"
    model_file.write_text(model_content)
    
    with pytest.raises(TextXSemanticError, match="cannot have both 'when' and 'unless'"):
        build_model(str(model_file))


def test_bridge_without_condition(tmp_path):
    """Test that bridges without conditions work normally."""
    model_content = """
Robot[ROS] FilterBot @ "localhost" {
    TOPICS
        data ["std_msgs/String", "/data"]
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[Topic] normal_bridge data -> "topic";
"""
    model_file = tmp_path / "test_no_condition.rbr"
    model_file.write_text(model_content)
    
    model, _ = build_model(str(model_file))
    bridge = model.bridges[0]
    
    assert bridge.has_condition is False
    assert bridge.condition_type is None
    assert bridge.condition is None


def test_condition_with_transform(tmp_path):
    """Test that conditions and transformations can coexist."""
    model_content = """
Robot[ROS] FilterBot @ "localhost" {
    TOPICS
        temp ["std_msgs/Float32", "/temperature"]
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[Topic] hot_temps temp -> "alerts/hot" {
    when: "msg.data > 100"
    transform: {
        temp_f: "msg.data * 1.8 + 32",
        alert: "'HOT'"
    }
};
"""
    model_file = tmp_path / "test_condition_transform.rbr"
    model_file.write_text(model_content)
    
    model, _ = build_model(str(model_file))
    bridge = model.bridges[0]
    
    assert bridge.has_condition is True
    assert bridge.condition_type == 'when'
    assert bridge.condition == "msg.data > 100"
    assert bridge.has_transform is True
    assert "temp_f" in bridge.transform_dict


def test_generated_code_contains_condition(when_model_content, tmp_path):
    """Verify that generated code contains condition checking logic."""
    from robocon.m2t.rosgen import GeneratorROS
    
    model_file = tmp_path / "test_gen_condition.rbr"
    model_file.write_text(when_model_content)
    
    model, _ = build_model(str(model_file))
    output_dir = tmp_path / "output"
    output_dir.mkdir()
    
    GeneratorROS.generate(model, str(output_dir))
    
    bridge_file = output_dir / "FilterBot_bridge.py"
    assert bridge_file.exists()
    
    content = bridge_file.read_text()
    assert "condition=" in content
    assert "condition_type=" in content
    assert "'when'" in content


def test_complex_condition_expression(tmp_path):
    """Test complex conditional expressions."""
    model_content = """
Robot[ROS] FilterBot @ "localhost" {
    TOPICS
        status ["std_msgs/String", "/status"]
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883
}

Bridge[Topic] complex_filter status -> "filtered" {
    when: "(msg.data.startswith('ERR') or msg.data.startswith('WARN')) and len(msg.data) > 10"
};
"""
    model_file = tmp_path / "test_complex.rbr"
    model_file.write_text(model_content)
    
    model, _ = build_model(str(model_file))
    bridge = model.bridges[0]
    
    assert bridge.has_condition is True
    assert "startswith" in bridge.condition
    assert "len(msg.data)" in bridge.condition
