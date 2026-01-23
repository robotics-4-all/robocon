import pytest
from robocon.utils import get_mm
from textx import TextXSemanticError

def test_tf_bridge_no_expansion():
    model_str = """
    Robot[ROS2] MyRobot @ "localhost" {
        TOPICS
    }
    
    Broker[Redis] MyBroker {
        host: "localhost",
        port: 6379
    }
    
    Bridge[TF] tf_bridge;
    """
    mm = get_mm()
    model = mm.model_from_str(model_str)
    
    # Check if TFBridge was NOT expanded
    assert len(model.bridges) == 1
    assert model.bridges[0].__class__.__name__ == 'TFBridge'
    assert model.bridges[0].name == 'tf_bridge'
    assert model.bridges[0].prefix == 'tf_bridge'

def test_tf_bridge_with_prefix():
    model_str = """
    Robot[ROS2] MyRobot @ "localhost" {
        TOPICS
    }
    
    Broker[Redis] MyBroker {
        host: "localhost",
        port: 6379
    }
    
    Bridge[TF] tf_bridge : "robot1/tf";
    """
    mm = get_mm()
    model = mm.model_from_str(model_str)
    
    assert len(model.bridges) == 1
    assert model.bridges[0].__class__.__name__ == 'TFBridge'
    assert model.bridges[0].prefix == 'robot1/tf'
