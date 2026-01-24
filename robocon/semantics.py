import re
from textx.exceptions import TextXSemanticError

class Config:
    """Helper class for nested configuration properties."""
    pass


def broker_processor(broker):
    """
    Object processor for MessageBroker.
    Handles nested properties and boolean conversions.
    """
    for p in broker.properties:
        value = p.value
        if value == 'True':
            value = True
        elif value == 'False':
            value = False

        parts = p.name.split('.')
        obj = broker
        for part in parts[:-1]:
            if not hasattr(obj, part):
                setattr(obj, part, Config())
            obj = getattr(obj, part)
        setattr(obj, parts[-1], value)


def bridge_processor(bridge):
    """
    Object processor for Bridge types.
    Sets direction, endpoints, and transformation metadata.
    """
    if hasattr(bridge, 'lhs') and bridge.lhs:
        if bridge.lhs.ros:
            bridge.direction = 'R2B'
            bridge.ros_endpoint = bridge.lhs.ros
            bridge.brokerURI = bridge.rhs.broker
        else:
            bridge.direction = 'B2R'
            bridge.ros_endpoint = bridge.rhs.ros
            bridge.brokerURI = bridge.lhs.broker

    # For template compatibility
    if bridge.__class__.__name__ == 'TopicBridge':
        bridge.topic = bridge.ros_endpoint
        
        # Handle transformations
        bridge.has_transform = hasattr(bridge, 'transforms') and len(bridge.transforms) > 0
        if bridge.has_transform:
            bridge.transform_dict = {t.target: t.expression for t in bridge.transforms}
        else:
            bridge.transform_dict = {}
        
        # Handle conditional filtering
        # Validate mutual exclusivity
        has_when = hasattr(bridge, 'when') and bridge.when is not None
        has_unless = hasattr(bridge, 'unless') and bridge.unless is not None
        
        if has_when and has_unless:
            raise TextXSemanticError(
                "Bridge cannot have both 'when' and 'unless' clauses. Use only one."
            )
        
        bridge.has_condition = has_when or has_unless
        if has_when:
            bridge.condition_type = 'when'
            bridge.condition = bridge.when.condition
        elif has_unless:
            bridge.condition_type = 'unless'
            bridge.condition = bridge.unless.condition
        else:
            bridge.condition_type = None
            bridge.condition = None
            
    elif bridge.__class__.__name__ == 'ServiceBridge':
        bridge.service = bridge.ros_endpoint
    elif bridge.__class__.__name__ == 'ActionBridge':
        bridge.action = bridge.ros_endpoint


def model_processor(model, metamodel):
    """
    Model processor for the entire RoboCon model.
    Expands NodeBridges into individual Topic/Service/Action bridges.
    """
    new_bridges = []
    for bridge in model.bridges:
        if bridge.__class__.__name__ == 'NodeBridge':
            node = bridge.node
            prefix = bridge.prefix if hasattr(bridge, 'prefix') and bridge.prefix else ""
            
            # Build mapping dictionaries
            topic_map = {}
            if hasattr(bridge, 'topic_maps') and bridge.topic_maps and hasattr(bridge.topic_maps, 'mappings'):
                for mapping in bridge.topic_maps.mappings:
                    topic_map[mapping.topic.name] = mapping.broker_uri
            
            service_map = {}
            if hasattr(bridge, 'service_maps') and bridge.service_maps and hasattr(bridge.service_maps, 'mappings'):
                for mapping in bridge.service_maps.mappings:
                    service_map[mapping.service.name] = mapping.broker_uri
            
            action_map = {}
            if hasattr(bridge, 'action_maps') and bridge.action_maps and hasattr(bridge.action_maps, 'mappings'):
                for mapping in bridge.action_maps.mappings:
                    action_map[mapping.action.name] = mapping.broker_uri
            
            # Expand publishes -> R2B TopicBridge
            for topic in getattr(node, 'publishes', []):
                tb = metamodel['TopicBridge']()
                tb.name = f"{bridge.name}_{topic.name}"
                tb.direction = 'R2B'
                tb.ros_endpoint = topic
                tb.topic = topic
                # Use custom mapping if provided, otherwise use prefix
                if topic.name in topic_map:
                    tb.brokerURI = topic_map[topic.name]
                else:
                    tb.brokerURI = f"{prefix}/{topic.name}" if prefix else topic.name
                bridge_processor(tb)
                new_bridges.append(tb)
            
            # Expand subscribes -> B2R TopicBridge
            for topic in getattr(node, 'subscribes', []):
                tb = metamodel['TopicBridge']()
                tb.name = f"{bridge.name}_{topic.name}"
                tb.direction = 'B2R'
                tb.ros_endpoint = topic
                tb.topic = topic
                # Use custom mapping if provided, otherwise use prefix
                if topic.name in topic_map:
                    tb.brokerURI = topic_map[topic.name]
                else:
                    tb.brokerURI = f"{prefix}/{topic.name}" if prefix else topic.name
                bridge_processor(tb)
                new_bridges.append(tb)
            
            # Expand services -> B2R ServiceBridge
            for service in getattr(node, 'services', []):
                sb = metamodel['ServiceBridge']()
                sb.name = f"{bridge.name}_{service.name}"
                sb.direction = 'B2R'
                sb.ros_endpoint = service
                sb.service = service
                # Use custom mapping if provided, otherwise use prefix
                if service.name in service_map:
                    sb.brokerURI = service_map[service.name]
                else:
                    sb.brokerURI = f"{prefix}/{service.name}" if prefix else service.name
                bridge_processor(sb)
                new_bridges.append(sb)
            
            # Expand actions -> B2R ActionBridge
            for action in getattr(node, 'actions', []):
                ab = metamodel['ActionBridge']()
                ab.name = f"{bridge.name}_{action.name}"
                ab.direction = 'B2R'
                ab.ros_endpoint = action
                ab.action = action
                # Use custom mapping if provided, otherwise use prefix
                if action.name in action_map:
                    ab.brokerURI = action_map[action.name]
                else:
                    ab.brokerURI = f"{prefix}/{action.name}" if prefix else action.name
                bridge_processor(ab)
                new_bridges.append(ab)
        elif bridge.__class__.__name__ == 'TFBridge':
            if not hasattr(bridge, 'prefix') or not bridge.prefix:
                bridge.prefix = bridge.name
            new_bridges.append(bridge)
        else:
            new_bridges.append(bridge)
    model.bridges = new_bridges


def validate_ros_uri(uri, entity_type="topic"):
    """
    Validate ROS URI against ROS naming conventions.
    
    Args:
        uri: The ROS URI to validate
        entity_type: Type of entity (topic, service, action) for error messages
        
    Raises:
        TextXSemanticError: If URI is invalid
    """
    if not uri:
        raise TextXSemanticError(f"ROS {entity_type} URI cannot be empty")
    
    if not uri.startswith('/'):
        raise TextXSemanticError(
            f"ROS {entity_type} URI must start with '/'. Got: {uri}"
        )
    
    if uri != '/' and uri.endswith('/'):
        raise TextXSemanticError(
            f"ROS {entity_type} URI cannot end with '/'. Got: {uri}"
        )
    
    if '//' in uri:
        raise TextXSemanticError(
            f"ROS {entity_type} URI cannot contain consecutive '/'. Got: {uri}"
        )
    
    # Check for invalid characters
    # Valid: alphanumeric, underscore, forward slash
    if not re.match(r'^/[a-zA-Z0-9_/]*$', uri):
        raise TextXSemanticError(
            f"ROS {entity_type} URI contains invalid characters. "
            f"Only alphanumeric, underscore, and '/' are allowed. Got: {uri}"
        )
    
    # Check that each segment starts with letter or underscore
    segments = uri.split('/')[1:]  # Skip empty first element from leading /
    for segment in segments:
        if segment and not re.match(r'^[a-zA-Z_]', segment):
            raise TextXSemanticError(
                f"ROS {entity_type} URI segments must start with a letter or underscore. "
                f"Invalid segment: '{segment}' in '{uri}'"
            )


def validate_model(model, metamodel):
    """
    Perform additional semantic validations on the model.
    """
    # Check if robot name is valid
    if not model.robot.name:
        raise TextXSemanticError("Robot must have a name.")
    
    # Validate ROS URIs for topics
    for topic in getattr(model.robot, 'topics', []):
        validate_ros_uri(topic.uri, "topic")
    
    # Validate ROS URIs for services
    for service in getattr(model.robot, 'services', []):
        validate_ros_uri(service.uri, "service")
    
    # Validate ROS URIs for actions
    for action in getattr(model.robot, 'actions', []):
        validate_ros_uri(action.uri, "action")
