import pytest
from robocon.semantics import (
    broker_processor, bridge_processor, model_processor, validate_model, Config
)
from textx import metamodel_from_str
from textx.exceptions import TextXSemanticError

@pytest.fixture
def mock_metamodel():
    grammar = """
    Model: robot=Robot bridges*=Bridge;
    Robot: 'Robot' name=ID;
    Bridge: TopicBridge;
    TopicBridge: 'Bridge' name=ID;
    """
    return metamodel_from_str(grammar)

def test_config_helper():
    conf = Config()
    conf.host = "localhost"
    assert conf.host == "localhost"

def test_broker_processor_nested_properties():
    class MockProperty:
        def __init__(self, name, value):
            self.name = name
            self.value = value

    class MockBroker:
        def __init__(self):
            self.properties = [
                MockProperty("network.host", "127.0.0.1"),
                MockProperty("network.port", "1883"),
                MockProperty("auth.enabled", "True")
            ]

    broker = MockBroker()
    broker_processor(broker)
    
    assert broker.network.host == "127.0.0.1"
    assert broker.network.port == "1883"
    assert broker.auth.enabled is True

def test_bridge_processor_r2b():
    class MockEndpoint:
        def __init__(self, ros=None, broker=None):
            self.ros = ros
            self.broker = broker

    class MockBridge:
        def __init__(self):
            self.lhs = MockEndpoint(ros="ros_topic")
            self.rhs = MockEndpoint(broker="broker_uri")
            self.__class__.__name__ = 'TopicBridge'

    bridge = MockBridge()
    bridge_processor(bridge)
    
    assert bridge.direction == 'R2B'
    assert bridge.ros_endpoint == "ros_topic"
    assert bridge.brokerURI == "broker_uri"
    assert bridge.has_transform is False

def test_validate_model_hook():
    class MockRobot:
        def __init__(self, name):
            self.name = name
    
    class MockModel:
        def __init__(self, name):
            self.robot = MockRobot(name)

    # Valid model
    model = MockModel("MyRobot")
    validate_model(model, None) # Should not raise
    
    # Invalid model
    invalid_model = MockModel("")
    with pytest.raises(TextXSemanticError, match="Robot must have a name"):
        validate_model(invalid_model, None)

def test_dotdict_logic():
    # Since DotDict is in the template, we can test its logic here by redefining it 
    # or just testing the behavior we expect in the generated code.
    # For now, let's verify that the bridge_processor correctly prepares the transform_dict.
    
    class MockTransform:
        def __init__(self, target, expression):
            self.target = target
            self.expression = expression

    class MockBridge:
        def __init__(self):
            self.transforms = [
                MockTransform("f1", "msg.a + 1"),
                MockTransform("f2", "msg.b * 2")
            ]
            self.__class__.__name__ = 'TopicBridge'
            self.lhs = None # To skip direction logic
            self.ros_endpoint = None # Add missing attribute

    bridge = MockBridge()
    bridge_processor(bridge)
    
    assert bridge.has_transform is True
    assert bridge.transform_dict == {"f1": "msg.a + 1", "f2": "msg.b * 2"}
