import os
import tempfile
from os.path import join
from textx import metamodel_from_file
from textx.exceptions import TextXSyntaxError, TextXSemanticError
import textx.scoping.providers as scoping_providers

from robocon.definitions import GRAMMAR_DIR


class Config:
    pass


def broker_processor(broker):
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
    elif bridge.__class__.__name__ == 'ServiceBridge':
        bridge.service = bridge.ros_endpoint
    elif bridge.__class__.__name__ == 'ActionBridge':
        bridge.action = bridge.ros_endpoint


def model_processor(model, metamodel):
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
                new_bridges.append(ab)
        else:
            new_bridges.append(bridge)
    model.bridges = new_bridges


def get_mm(debug=False, global_scope=True):
    mm= metamodel_from_file(
        join(GRAMMAR_DIR, 'robocon.tx'),
        global_repository=global_scope,
        debug=debug
    )
    mm.register_obj_processors({
        'MessageBroker': broker_processor,
        'TopicBridge': bridge_processor,
        'ServiceBridge': bridge_processor,
        'ActionBridge': bridge_processor,
        'TFBridge': bridge_processor
    })
    mm.register_model_processor(model_processor)

    def importURI_to_scope_name(import_obj):
        # this method is responsible to deduce the module name in the
        # language from the importURI string
        # e.g. here: import "file.ext" --> module name "file".
        return import_obj.importURI.split('.')[0]

    def conv(i):
        return i.replace(".", "/") + ".rbr"

    mm.register_scope_providers(
        {
            "*.*": scoping_providers.FQNImportURI(
                importAs=True,
                # importURI_to_scope_name=importURI_to_scope_name
                # importURI_converter=conv
            ),
            "TopicEndpoint.ros": scoping_providers.PlainName(),
            "ServiceEndpoint.ros": scoping_providers.PlainName(),
            "ActionEndpoint.ros": scoping_providers.PlainName(),
            "TFEndpoint.ros": scoping_providers.PlainName(),
            "NodeBridge.node": scoping_providers.PlainName(),
            "ROSNode.publishes": scoping_providers.PlainName(),
            "ROSNode.subscribes": scoping_providers.PlainName(),
            "ROSNode.services": scoping_providers.PlainName(),
            "ROSNode.actions": scoping_providers.PlainName(),
            "TopicMapping.topic": scoping_providers.PlainName(),
            "ServiceMapping.service": scoping_providers.PlainName(),
            "ActionMapping.action": scoping_providers.PlainName(),
        }
    )

    return mm


def build_model(model_fpath):
    mm = get_mm(global_scope=True)
    model = mm.model_from_file(model_fpath)
    # print(model._tx_loaded_models)
    reg_models = mm._tx_model_repository.all_models.filename_to_model
    mimports = [val for key, val in reg_models.items() if val != model]
    return (model, mimports)


def check_model_errors(model_input: str):
    """
    Parse and validate a RoboCon model, returning all detected errors.
    
    This function validates both syntactic and semantic rules of the model.
    
    Args:
        model_input: Either a file path (str) to a .rbr file, or model content as string
        
    Returns:
        list: List of error dictionaries with the following schema:
            {
                "type": "syntax" or "semantic",
                "message": str,
                "line": int,
                "column": int
            }
        Returns empty list if no errors are found.
    """
    errors = []
    temp_file = None
    
    try:
        # Determine if input is a file path or string content
        if os.path.exists(model_input):
            # Input is a file path
            model_path = model_input
        else:
            # Input is model string content - create temporary file
            temp_file = tempfile.NamedTemporaryFile(
                mode='w',
                suffix='.rbr',
                delete=False,
                encoding='utf-8'
            )
            temp_file.write(model_input)
            temp_file.close()
            model_path = temp_file.name
        
        # Try to build the model to catch all errors
        mm = get_mm(global_scope=True)
        model = mm.model_from_file(model_path)
        
    except TextXSyntaxError as e:
        # Syntax error occurred
        error = {
            "type": "syntax",
            "message": e.message if hasattr(e, 'message') else str(e),
            "line": e.line if hasattr(e, 'line') else 0,
            "column": e.col if hasattr(e, 'col') else 0
        }
        errors.append(error)
        
    except TextXSemanticError as e:
        # Semantic error occurred
        error = {
            "type": "semantic",
            "message": e.message if hasattr(e, 'message') else str(e),
            "line": e.line if hasattr(e, 'line') else 0,
            "column": e.col if hasattr(e, 'col') else 0
        }
        errors.append(error)
        
    except Exception as e:
        # Catch any other unexpected errors
        error = {
            "type": "unknown",
            "message": str(e),
            "line": 0,
            "column": 0
        }
        errors.append(error)
        
    finally:
        # Clean up temporary file if created
        if temp_file and os.path.exists(temp_file.name):
            try:
                os.unlink(temp_file.name)
            except Exception:
                pass  # Ignore cleanup errors
    
    return errors


def get_grammar(debug=False):
    with open(join(GRAMMAR_DIR, 'robocon.tx')) as f:
        return f.read()