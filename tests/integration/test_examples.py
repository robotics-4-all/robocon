"""
Integration tests using all example models.
"""
import pytest
import os
from pathlib import Path
from robocon.utils import build_model
from robocon.m2t.rosgen import GeneratorROS
from robocon.m2t.ros2gen import GeneratorROS2


def get_example_files():
    """Get all .rbr files from the examples directory."""
    examples_dir = Path(__file__).parent.parent.parent / "examples"
    return list(examples_dir.glob("*.rbr"))


@pytest.mark.parametrize("example_file", get_example_files(), ids=lambda p: p.name)
def test_example_validates(example_file):
    """Test that each example model validates successfully."""
    model, imports = build_model(str(example_file))
    assert model is not None
    assert model.robot is not None
    assert model.broker is not None


@pytest.mark.parametrize("example_file", get_example_files(), ids=lambda p: p.name)
def test_example_has_bridges(example_file):
    """Test that each example model has bridges defined or expanded."""
    model, imports = build_model(str(example_file))
    assert len(model.bridges) > 0


def test_ros_examples_generate(temp_dir):
    """Test that ROS examples generate valid code."""
    examples_dir = Path(__file__).parent.parent.parent / "examples"
    ros_examples = [
        "ros_model.rbr",
        "ros_turtlesim.rbr",
        "ros_mavros.rbr",
        "ros_navigation.rbr"
    ]
    
    for example in ros_examples:
        example_path = examples_dir / example
        if not example_path.exists():
            continue
            
        model, imports = build_model(str(example_path))
        if model.robot.type != 'ROS':
            continue
        
        out_dir = os.path.join(temp_dir, f"gen_{example.replace('.rbr', '')}")
        GeneratorROS.generate(model, out_dir)
        
        # Verify generation succeeded
        assert os.path.exists(out_dir)
        bridge_file = os.path.join(out_dir, f"{model.robot.name}_bridge.py")
        assert os.path.exists(bridge_file)


def test_ros2_examples_generate(temp_dir):
    """Test that ROS2 examples generate valid code."""
    examples_dir = Path(__file__).parent.parent.parent / "examples"
    ros2_examples = [
        "ros2_model.rbr",
        "complex_model.rbr",
        "turtlesim.rbr",
        "drone_system.rbr",
        "navigation_stack.rbr",
        "manipulation_system.rbr",
        "perception_system.rbr",
        "slam_system.rbr"
    ]
    
    for example in ros2_examples:
        example_path = examples_dir / example
        if not example_path.exists():
            continue
            
        model, imports = build_model(str(example_path))
        if model.robot.type != 'ROS2':
            continue
        
        out_dir = os.path.join(temp_dir, f"gen_{example.replace('.rbr', '')}")
        GeneratorROS2.generate(model, out_dir)
        
        # Verify generation succeeded
        assert os.path.exists(out_dir)
        bridge_file = os.path.join(out_dir, f"{model.robot.name}_bridges.py")
        assert os.path.exists(bridge_file)


def test_all_generated_code_is_valid_python(temp_dir):
    """Test that all generated code is syntactically valid Python."""
    examples_dir = Path(__file__).parent.parent.parent / "examples"
    
    for example_file in examples_dir.glob("*.rbr"):
        model, imports = build_model(str(example_file))
        out_dir = os.path.join(temp_dir, f"gen_{example_file.stem}")
        
        if model.robot.type == 'ROS':
            GeneratorROS.generate(model, out_dir)
            bridge_file = os.path.join(out_dir, f"{model.robot.name}_bridge.py")
        else:
            GeneratorROS2.generate(model, out_dir)
            bridge_file = os.path.join(out_dir, f"{model.robot.name}_bridges.py")
        
        # Verify Python syntax
        with open(bridge_file, 'r') as f:
            code = f.read()
            compile(code, bridge_file, 'exec')


def test_node_bridge_expansion():
    """Test that NodeBridge properly expands to individual bridges."""
    examples_dir = Path(__file__).parent.parent.parent / "examples"
    turtlesim_file = examples_dir / "ros_turtlesim.rbr"
    
    if not turtlesim_file.exists():
        pytest.skip("ros_turtlesim.rbr not found")
    
    model, imports = build_model(str(turtlesim_file))
    
    # The model uses NodeBridge which should expand
    assert len(model.bridges) > 1
    
    # Check that bridges have correct directions
    r2b_bridges = [b for b in model.bridges if b.direction == 'R2B']
    b2r_bridges = [b for b in model.bridges if b.direction == 'B2R']
    
    assert len(r2b_bridges) > 0  # Should have publishes
    assert len(b2r_bridges) > 0  # Should have subscribes and services
