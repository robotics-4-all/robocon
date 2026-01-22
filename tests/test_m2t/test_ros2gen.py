"""
Tests for the ROS2 code generator.
"""
import pytest
import os
from unittest.mock import patch
from robocon.m2t.ros2gen import GeneratorROS2


class TestGeneratorROS2:
    """Tests for the GeneratorROS2 class."""
    
    def test_generator_has_templates(self):
        """Test that GeneratorROS2 has template references."""
        assert GeneratorROS2.bridge_tpl is not None
        assert GeneratorROS2.reqs_tpl is not None
    
    def test_generator_has_dependencies(self):
        """Test that GeneratorROS2 defines Python dependencies."""
        assert len(GeneratorROS2.PY_DEPS) > 0
        assert ('commlib-py', '>=', '0.11.2') in GeneratorROS2.PY_DEPS
        assert ('ros2-msg-transform', '>=', '0.2.2') in GeneratorROS2.PY_DEPS
    
    def test_generate_creates_output_directory(self, temp_dir, metamodel, sample_ros2_model):
        """Test that generate creates the output directory if it doesn't exist."""
        model = metamodel.model_from_str(sample_ros2_model)
        out_dir = os.path.join(temp_dir, "gen_output")
        
        assert not os.path.exists(out_dir)
        GeneratorROS2.generate(model, out_dir)
        assert os.path.exists(out_dir)
    
    def test_generate_creates_bridge_file(self, temp_dir, metamodel, sample_ros2_model):
        """Test that generate creates a bridge Python file."""
        model = metamodel.model_from_str(sample_ros2_model)
        out_dir = os.path.join(temp_dir, "gen_output")
        
        GeneratorROS2.generate(model, out_dir)
        
        expected_file = os.path.join(out_dir, f"{model.robot.name}_bridges.py")
        assert os.path.exists(expected_file)
    
    def test_generate_creates_requirements_file(self, temp_dir, metamodel, sample_ros2_model):
        """Test that generate creates requirements.txt."""
        model = metamodel.model_from_str(sample_ros2_model)
        out_dir = os.path.join(temp_dir, "gen_output")
        
        GeneratorROS2.generate(model, out_dir)
        
        req_file = os.path.join(out_dir, "requirements.txt")
        assert os.path.exists(req_file)
    
    def test_generate_sets_executable_permissions(self, temp_dir, metamodel, sample_ros2_model):
        """Test that generated file has executable permissions."""
        model = metamodel.model_from_str(sample_ros2_model)
        out_dir = os.path.join(temp_dir, "gen_output")
        
        GeneratorROS2.generate(model, out_dir)
        
        bridge_file = os.path.join(out_dir, f"{model.robot.name}_bridges.py")
        # Check that file has execute permissions
        import stat
        file_stat = os.stat(bridge_file)
        assert file_stat.st_mode & stat.S_IXUSR  # User execute
    
    def test_generate_validates_ros2_type(self, temp_dir, metamodel, sample_ros_model, capsys):
        """Test that generate validates ROS2 type."""
        model = metamodel.model_from_str(sample_ros_model)
        out_dir = os.path.join(temp_dir, "gen_output")
        
        GeneratorROS2.generate(model, out_dir)
        
        captured = capsys.readouterr()
        assert "Did not find any ROS2 System definition" in captured.out
    
    def test_gen_requirements_content(self, temp_dir):
        """Test that gen_requirements creates correct content."""
        GeneratorROS2.gen_requirements(temp_dir)
        
        req_file = os.path.join(temp_dir, "requirements.txt")
        assert os.path.exists(req_file)
        
        with open(req_file, 'r') as f:
            content = f.read()
            assert "commlib-py" in content
            assert "ros2-msg-transform" in content
    
    def test_report_prints_model_info(self, metamodel, sample_ros2_model, capsys):
        """Test that report prints model information."""
        model = metamodel.model_from_str(sample_ros2_model)
        
        GeneratorROS2.report(model)
        
        captured = capsys.readouterr()
        assert "TestRobot2" in captured.out
        assert "Bridge:" in captured.out
    
    def test_generated_code_is_valid_python(self, temp_dir, metamodel, sample_ros2_model):
        """Test that generated code is syntactically valid Python."""
        model = metamodel.model_from_str(sample_ros2_model)
        out_dir = os.path.join(temp_dir, "gen_output")
        
        GeneratorROS2.generate(model, out_dir)
        
        bridge_file = os.path.join(out_dir, f"{model.robot.name}_bridges.py")
        
        # Try to compile the generated Python file
        with open(bridge_file, 'r') as f:
            code = f.read()
            compile(code, bridge_file, 'exec')  # Should not raise SyntaxError
