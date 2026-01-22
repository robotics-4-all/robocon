"""
Tests for the CLI commands.
"""
import pytest
import os
from click.testing import CliRunner
from robocon.cli.cli import cli


class TestValidateCommand:
    """Tests for the validate CLI command."""
    
    def test_validate_valid_ros_model(self, model_file):
        """Test validate command with a valid ROS model."""
        runner = CliRunner()
        result = runner.invoke(cli, ['validate', model_file])
        
        assert result.exit_code == 0
        assert "validation success" in result.output.lower()
    
    def test_validate_valid_ros2_model(self, ros2_model_file):
        """Test validate command with a valid ROS2 model."""
        runner = CliRunner()
        result = runner.invoke(cli, ['validate', ros2_model_file])
        
        assert result.exit_code == 0
        assert "validation success" in result.output.lower()
    
    def test_validate_invalid_model(self, temp_dir, write_model_file, invalid_model):
        """Test validate command with an invalid model."""
        invalid_file = write_model_file(invalid_model, "invalid.rbr")
        
        runner = CliRunner()
        result = runner.invoke(cli, ['validate', invalid_file])
        
        # Should fail with non-zero exit code
        assert result.exit_code != 0
    
    def test_validate_nonexistent_file(self):
        """Test validate command with non-existent file."""
        runner = CliRunner()
        result = runner.invoke(cli, ['validate', '/nonexistent/file.rbr'])
        
        assert result.exit_code != 0


class TestGenCommand:
    """Tests for the gen CLI command."""
    
    def test_gen_ros_model_auto_detect(self, temp_dir, model_file):
        """Test gen command with ROS model using auto-detection."""
        runner = CliRunner()
        out_dir = os.path.join(temp_dir, "output")
        
        result = runner.invoke(cli, ['gen', model_file, '-o', out_dir])
        
        assert result.exit_code == 0
        assert "Auto-detected ROS" in result.output
        assert os.path.exists(out_dir)
    
    def test_gen_ros2_model_auto_detect(self, temp_dir, ros2_model_file):
        """Test gen command with ROS2 model using auto-detection."""
        runner = CliRunner()
        out_dir = os.path.join(temp_dir, "output")
        
        result = runner.invoke(cli, ['gen', ros2_model_file, '-o', out_dir])
        
        assert result.exit_code == 0
        assert "Auto-detected ROS2" in result.output
        assert os.path.exists(out_dir)
    
    def test_gen_explicit_ros_generator(self, temp_dir, model_file):
        """Test gen command with explicit ROS generator."""
        runner = CliRunner()
        out_dir = os.path.join(temp_dir, "output")
        
        result = runner.invoke(cli, ['gen', model_file, 'ros', '-o', out_dir])
        
        assert result.exit_code == 0
        assert os.path.exists(out_dir)
    
    def test_gen_explicit_ros2_generator(self, temp_dir, ros2_model_file):
        """Test gen command with explicit ROS2 generator."""
        runner = CliRunner()
        out_dir = os.path.join(temp_dir, "output")
        
        result = runner.invoke(cli, ['gen', ros2_model_file, 'ros2', '-o', out_dir])
        
        assert result.exit_code == 0
        assert os.path.exists(out_dir)
    
    def test_gen_creates_bridge_file(self, temp_dir, model_file):
        """Test that gen command creates bridge file."""
        runner = CliRunner()
        out_dir = os.path.join(temp_dir, "output")
        
        result = runner.invoke(cli, ['gen', model_file, '-o', out_dir])
        
        assert result.exit_code == 0
        # Should have a _bridge.py or _bridges.py file
        files = os.listdir(out_dir)
        assert any('bridge' in f and f.endswith('.py') for f in files)
    
    def test_gen_creates_requirements(self, temp_dir, model_file):
        """Test that gen command creates requirements.txt."""
        runner = CliRunner()
        out_dir = os.path.join(temp_dir, "output")
        
        result = runner.invoke(cli, ['gen', model_file, '-o', out_dir])
        
        assert result.exit_code == 0
        req_file = os.path.join(out_dir, "requirements.txt")
        assert os.path.exists(req_file)
    
    def test_gen_unknown_generator(self, temp_dir, model_file):
        """Test gen command with unknown generator."""
        runner = CliRunner()
        out_dir = os.path.join(temp_dir, "output")
        
        result = runner.invoke(cli, ['gen', model_file, 'unknown', '-o', out_dir])
        
        assert "Unknown generator" in result.output
    
    def test_gen_default_output_directory(self, model_file):
        """Test gen command uses default output directory."""
        runner = CliRunner()
        
        with runner.isolated_filesystem():
            result = runner.invoke(cli, ['gen', model_file])
            # Default is "gen"
            assert os.path.exists("gen") or result.exit_code == 0
    
    def test_gen_case_insensitive_generator(self, temp_dir, model_file):
        """Test that generator name is case insensitive."""
        runner = CliRunner()
        out_dir = os.path.join(temp_dir, "output")
        
        # Try with different case variations
        result = runner.invoke(cli, ['gen', model_file, 'ROS', '-o', out_dir])
        assert result.exit_code == 0
