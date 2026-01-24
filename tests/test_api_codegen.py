import os
import tarfile
import io
import pytest
from fastapi.testclient import TestClient
from api.api import api

client = TestClient(api)

@pytest.fixture
def api_headers():
    return {"X-API-Key": "123123"}

def test_generate_code_ros(api_headers, sample_ros_model):
    """Test code generation for a ROS model."""
    data = {
        "name": "test_ros_project",
        "model": sample_ros_model
    }
    response = client.post("/generate", json=data, headers=api_headers)
    
    assert response.status_code == 200
    assert response.headers["content-type"] == "application/x-tar"
    assert "test_ros_project_generated_code.tar.gz" in response.headers["content-disposition"]
    
    # Verify tarball content
    with tarfile.open(fileobj=io.BytesIO(response.content), mode="r:gz") as tar:
        names = tar.getnames()
        assert "TestRobot_bridge.py" in names
        assert "requirements.txt" in names

def test_generate_code_ros2(api_headers, sample_ros2_model):
    """Test code generation for a ROS2 model."""
    data = {
        "name": "test_ros2_project",
        "model": sample_ros2_model
    }
    response = client.post("/generate", json=data, headers=api_headers)
    
    assert response.status_code == 200
    assert response.headers["content-type"] == "application/x-tar"
    
    # Verify tarball content
    with tarfile.open(fileobj=io.BytesIO(response.content), mode="r:gz") as tar:
        names = tar.getnames()
        assert "TestRobot2_bridges.py" in names
        assert "requirements.txt" in names

def test_generate_code_invalid_model(api_headers):
    """Test code generation with an invalid model."""
    data = {
        "name": "invalid_project",
        "model": "invalid model content"
    }
    response = client.post("/generate", json=data, headers=api_headers)
    
    assert response.status_code == 400
    assert "Model validation failed" in response.json()["detail"]

def test_generate_code_empty_model(api_headers):
    """Test code generation with empty model content."""
    data = {
        "name": "empty_project",
        "model": ""
    }
    response = client.post("/generate", json=data, headers=api_headers)
    
    assert response.status_code == 400
    assert "Model content cannot be empty" in response.json()["detail"]

def test_generate_code_no_api_key(sample_ros_model):
    """Test code generation without API key."""
    data = {
        "name": "test_project",
        "model": sample_ros_model
    }
    response = client.post("/generate", json=data)
    
    assert response.status_code == 401


def test_generate_code_file_ros(api_headers, sample_ros_model):
    """Test code generation from an uploaded ROS model file."""
    files = {
        "file": ("test_robot.rbr", sample_ros_model, "text/plain")
    }
    response = client.post("/generate/file", files=files, headers=api_headers)
    
    assert response.status_code == 200
    assert response.headers["content-type"] == "application/x-tar"
    assert "test_robot_generated_code.tar.gz" in response.headers["content-disposition"]
    
    # Verify tarball content
    with tarfile.open(fileobj=io.BytesIO(response.content), mode="r:gz") as tar:
        names = tar.getnames()
        assert "TestRobot_bridge.py" in names
        assert "requirements.txt" in names


def test_generate_code_file_ros2(api_headers, sample_ros2_model):
    """Test code generation from an uploaded ROS2 model file."""
    files = {
        "file": ("test_robot2.rbr", sample_ros2_model, "text/plain")
    }
    response = client.post("/generate/file", files=files, headers=api_headers)
    
    assert response.status_code == 200
    assert response.headers["content-type"] == "application/x-tar"
    assert "test_robot2_generated_code.tar.gz" in response.headers["content-disposition"]
    
    # Verify tarball content
    with tarfile.open(fileobj=io.BytesIO(response.content), mode="r:gz") as tar:
        names = tar.getnames()
        assert "TestRobot2_bridges.py" in names
        assert "requirements.txt" in names


def test_generate_code_file_invalid(api_headers):
    """Test code generation from an invalid uploaded model file."""
    files = {
        "file": ("invalid.rbr", "invalid content", "text/plain")
    }
    response = client.post("/generate/file", files=files, headers=api_headers)
    
    assert response.status_code == 400
    assert "Model validation failed" in response.json()["detail"]

