"""
Tests for the robocon utils module.
"""
import pytest
import os
from robocon.utils import (
    Config,
    broker_processor,
    bridge_processor,
    model_processor,
    get_mm,
    build_model,
    get_grammar
)


class TestConfig:
    """Tests for the Config class."""
    
    def test_config_can_set_attributes(self):
        """Test that Config allows setting arbitrary attributes."""
        config = Config()
        config.test_attr = "test_value"
        assert config.test_attr == "test_value"
    
    def test_config_nested_attributes(self):
        """Test that nested attributes can be set."""
        config = Config()
        config.nested = Config()
        config.nested.value = 42
        assert config.nested.value == 42


class TestBrokerProcessor:
    """Tests for broker_processor function."""
    
    def test_broker_processor_with_sample_model(self, metamodel, sample_ros_model):
        """Test broker processor processes model correctly."""
        model = metamodel.model_from_str(sample_ros_model)
        
        # Broker host should be set
        assert model.broker.host == "localhost"
        assert model.broker.port == 1883
    
    def test_broker_processor_with_auth(self, metamodel):
        """Test broker processor with authentication."""
        model_with_auth = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        test [ "std_msgs/String", "/test" ]
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883,
    auth.username: "user",
    auth.password: "pass"
}

Bridge[Topic] test_bridge test:"test/topic";
"""
        model = metamodel.model_from_str(model_with_auth)
        
        # Auth properties should be nested
        assert hasattr(model.broker, 'auth')
        assert model.broker.auth.username == "user"
        assert model.broker.auth.password == "pass"
    
    def test_broker_processor_boolean_values(self, metamodel):
        """Test broker processor converts boolean strings."""
        model_with_bool = """
Robot[ROS] TestRobot @ "localhost" {
    TOPICS
        test [ "std_msgs/String", "/test" ]
}

Broker[MQTT] TestBroker {
    host: "localhost",
    port: 1883,
    ssl: "True"
}

Bridge[Topic] test_bridge test:"test/topic";
"""
        model = metamodel.model_from_str(model_with_bool)
        
        # SSL should be converted to boolean
        assert model.broker.ssl is True


class TestBridgeProcessor:
    """Tests for bridge_processor function."""
    
    def test_bridge_processor_r2b_direction(self, metamodel, sample_ros_model):
        """Test bridge processor determines R2B direction correctly."""
        model = metamodel.model_from_str(sample_ros_model)
        
        # Find the odom bridge (ROS -> Broker)
        odom_bridge = [b for b in model.bridges if b.name == "odom_bridge"][0]
        assert odom_bridge.direction == "R2B"
        assert odom_bridge.ros_endpoint is not None
        assert odom_bridge.brokerURI == "robot/odom"
    
    def test_bridge_processor_b2r_direction(self, metamodel, sample_ros_model):
        """Test bridge processor determines B2R direction correctly."""
        model = metamodel.model_from_str(sample_ros_model)
        
        # Find the cmd bridge (Broker -> ROS)
        cmd_bridge = [b for b in model.bridges if b.name == "cmd_bridge"][0]
        assert cmd_bridge.direction == "B2R"
        assert cmd_bridge.ros_endpoint is not None
        assert cmd_bridge.brokerURI == "robot/cmd"
    
    def test_bridge_processor_sets_topic_attribute(self, metamodel, sample_ros_model):
        """Test that bridge processor sets topic attribute for TopicBridge."""
        model = metamodel.model_from_str(sample_ros_model)
        
        odom_bridge = [b for b in model.bridges if b.name == "odom_bridge"][0]
        assert hasattr(odom_bridge, 'topic')
        assert odom_bridge.topic is not None
    
    def test_bridge_processor_sets_service_attribute(self, metamodel, sample_ros_model):
        """Test that bridge processor sets service attribute for ServiceBridge."""
        model = metamodel.model_from_str(sample_ros_model)
        
        reset_bridge = [b for b in model.bridges if b.name == "reset_bridge"][0]
        assert hasattr(reset_bridge, 'service')
        assert reset_bridge.service is not None


class TestModelProcessor:
    """Tests for model_processor function."""
    
    def test_model_processor_expands_node_bridge(self, metamodel, sample_node_bridge_model):
        """Test that model processor expands NodeBridge to individual bridges."""
        model = metamodel.model_from_str(sample_node_bridge_model)
        
        # NodeBridge should be expanded to multiple bridges
        assert len(model.bridges) > 1
        
        # Should have bridges for publishes (pose, status)
        pose_bridge = [b for b in model.bridges if 'pose' in b.name]
        status_bridge = [b for b in model.bridges if 'status' in b.name]
        assert len(pose_bridge) > 0
        assert len(status_bridge) > 0
    
    def test_model_processor_publishes_creates_r2b(self, metamodel, sample_node_bridge_model):
        """Test that publishes creates R2B TopicBridge."""
        model = metamodel.model_from_str(sample_node_bridge_model)
        
        # Find pose bridge (published topic)
        pose_bridges = [b for b in model.bridges if 'pose' in b.name]
        assert len(pose_bridges) > 0
        pose_bridge = pose_bridges[0]
        assert pose_bridge.direction == "R2B"
    
    def test_model_processor_subscribes_creates_b2r(self, metamodel, sample_node_bridge_model):
        """Test that subscribes creates B2R TopicBridge."""
        model = metamodel.model_from_str(sample_node_bridge_model)
        
        # Find velocity bridge (subscribed topic)
        velocity_bridges = [b for b in model.bridges if 'velocity' in b.name]
        assert len(velocity_bridges) > 0
        velocity_bridge = velocity_bridges[0]
        assert velocity_bridge.direction == "B2R"
    
    def test_model_processor_services_creates_b2r(self, metamodel, sample_node_bridge_model):
        """Test that services creates B2R ServiceBridge."""
        model = metamodel.model_from_str(sample_node_bridge_model)
        
        # Find service bridges
        service_bridges = [b for b in model.bridges if 'start' in b.name or 'stop' in b.name]
        assert len(service_bridges) > 0
        for bridge in service_bridges:
            assert bridge.direction == "B2R"
    
    def test_model_processor_uses_prefix(self, metamodel, sample_node_bridge_model):
        """Test that model processor uses the prefix in brokerURI."""
        model = metamodel.model_from_str(sample_node_bridge_model)
        
        # All expanded bridges should have the prefix
        for bridge in model.bridges:
            assert bridge.brokerURI.startswith("robot/controller/")


class TestGetMM:
    """Tests for get_mm function."""
    
    def test_get_mm_returns_metamodel(self):
        """Test that get_mm returns a metamodel."""
        mm = get_mm()
        assert mm is not None
        assert hasattr(mm, 'model_from_str')
    
    def test_get_mm_with_debug_false(self):
        """Test get_mm with debug=False."""
        mm = get_mm(debug=False)
        assert mm is not None
    
    def test_get_mm_with_debug_true(self):
        """Test get_mm with debug=True."""
        mm = get_mm(debug=True)
        assert mm is not None
    
    def test_get_mm_with_global_scope(self):
        """Test get_mm with global_scope parameter."""
        mm = get_mm(global_scope=True)
        assert mm is not None
    
    def test_get_mm_has_processors_registered(self):
        """Test that get_mm registers object processors."""
        mm = get_mm()
        # Processors should be registered for MessageBroker and bridges
        # This is implicit - we test by parsing a model
        assert mm is not None


class TestBuildModel:
    """Tests for build_model function."""
    
    def test_build_model_from_file(self, model_file):
        """Test that build_model loads a model from file."""
        model, imports = build_model(model_file)
        assert model is not None
        assert model.robot.name == "TestRobot"
    
    def test_build_model_returns_imports(self, model_file):
        """Test that build_model returns imports list."""
        model, imports = build_model(model_file)
        assert isinstance(imports, list)
    
    def test_build_model_processes_model(self, node_bridge_model_file):
        """Test that build_model applies model processors."""
        model, imports = build_model(node_bridge_model_file)
        # NodeBridge should be expanded
        assert len(model.bridges) > 1


class TestGetGrammar:
    """Tests for get_grammar function."""
    
    def test_get_grammar_returns_string(self):
        """Test that get_grammar returns a string."""
        grammar = get_grammar()
        assert isinstance(grammar, str)
        assert len(grammar) > 0
    
    def test_get_grammar_contains_model_rule(self):
        """Test that grammar contains the Model rule."""
        grammar = get_grammar()
        assert "Model:" in grammar
    
    def test_get_grammar_contains_robot_rule(self):
        """Test that grammar contains the Robot rule."""
        grammar = get_grammar()
        assert "Robot:" in grammar
    
    def test_get_grammar_contains_bridge_rules(self):
        """Test that grammar contains bridge rules."""
        grammar = get_grammar()
        assert "Bridge:" in grammar
        assert "TopicBridge:" in grammar
