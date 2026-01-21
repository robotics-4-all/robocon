"""
Tests for the ROS code generator.
"""
import pytest
import os
from unittest.mock import patch, Mock
from robocon.m2t.rosgen import GeneratorROS


class TestGeneratorROS:
    """Tests for the GeneratorROS class."""
    
    def test_generator_has_templates(self):
        """Test that GeneratorROS has template references."""
        assert GeneratorROS.bridge_tpl is not None
        assert GeneratorROS.reqs_tpl is not None
    
    def test_generator_has_dependencies(self):
        """Test that GeneratorROS defines Python dependencies."""
        assert len(GeneratorROS.PY_DEPS) > 0
        assert ('commlib-py', '>=', '0.11.2') in GeneratorROS.PY_DEPS
        assert ('ros-msg-transform', '>=', '0.1.0') in GeneratorROS.PY_DEPS
    
    def test_generate_creates_output_directory(self, temp_dir, metamodel, sample_ros_model):
        """Test that generate creates the output directory if it doesn't exist."""
        model = metamodel.model_from_str(sample_ros_model)
        out_dir = os.path.join(temp_dir, "gen_output")
        
        assert not os.path.exists(out_dir)
        GeneratorROS.generate(model, out_dir)
        assert os.path.exists(out_dir)
    
    def test_generate_creates_bridge_file(self, temp_dir, metamodel, sample_ros_model):
        """Test that generate creates a bridge Python file."""
        model = metamodel.model_from_str(sample_ros_model)
        out_dir = os.path.join(temp_dir, "gen_output")
        
        GeneratorROS.generate(model, out_dir)
        
        expected_file = os.path.join(out_dir, f"{model.robot.name}_bridge.py")
        assert os.path.exists(expected_file)
    
    def test_generate_creates_requirements_file(self, temp_dir, metamodel, sample_ros_model):
        """Test that generate creates requirements.txt."""
        model = metamodel.model_from_str(sample_ros_model)
        out_dir = os.path.join(temp_dir, "gen_output")
        
        GeneratorROS.generate(model, out_dir)
        
        req_file = os.path.join(out_dir, "requirements.txt")
        assert os.path.exists(req_file)
    
    def test_generate_sets_executable_permissions(self, temp_dir, metamodel, sample_ros_model):
        """Test that generated file has executable permissions."""
        model = metamodel.model_from_str(sample_ros_model)
        out_dir = os.path.join(temp_dir, "gen_output")
        
        GeneratorROS.generate(model, out_dir)
        
        bridge_file = os.path.join(out_dir, f"{model.robot.name}_bridge.py")
        # Check that file has execute permissions (509 = 0o775)
        import stat
        file_stat = os.stat(bridge_file)
        assert file_stat.st_mode & stat.S_IXUSR  # User execute
    
    def test_generate_validates_ros_type(self, temp_dir, metamodel, sample_ros2_model, capsys):
        """Test that generate validates ROS type."""
        model = metamodel.model_from_str(sample_ros2_model)
        out_dir = os.path.join(temp_dir, "gen_output")
        
        GeneratorROS.generate(model, out_dir)
        
        captured = capsys.readouterr()
        assert "Did not find any ROS System definition" in captured.out
    
    def test_gen_requirements_content(self, temp_dir):
        """Test that gen_requirements creates correct content."""
        GeneratorROS.gen_requirements(temp_dir)
        
        req_file = os.path.join(temp_dir, "requirements.txt")
        assert os.path.exists(req_file)
        
        with open(req_file, 'r') as f:
            content = f.read()
            assert "commlib-py" in content
            assert "ros-msg-transform" in content
    
    def test_report_prints_model_info(self, metamodel, sample_ros_model, capsys):
        """Test that report prints model information."""
        model = metamodel.model_from_str(sample_ros_model)
        
        GeneratorROS.report(model)
        
        captured = capsys.readouterr()
        assert "TestRobot" in captured.out
        assert "Bridge:" in captured.out
    
    def test_generated_code_is_valid_python(self, temp_dir, metamodel, sample_ros_model):
        """Test that generated code is syntactically valid Python."""
        model = metamodel.model_from_str(sample_ros_model)
        out_dir = os.path.join(temp_dir, "gen_output")
        
        GeneratorROS.generate(model, out_dir)
        
        bridge_file = os.path.join(out_dir, f"{model.robot.name}_bridge.py")
        
        # Try to compile the generated Python file
        with open(bridge_file, 'r') as f:
            code = f.read()
            compile(code, bridge_file, 'exec')  # Should not raise SyntaxError
