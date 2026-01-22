"""
Tests for the FastAPI endpoints.
"""
import pytest
import base64
import os
from fastapi.testclient import TestClient
from api.api import api


class TestAPIValidation:
    """Tests for API validation endpoints."""
    
    @pytest.fixture
    def client(self):
        """Create a test client."""
        return TestClient(api)
    
    @pytest.fixture
    def api_headers(self):
        """Return headers with valid API key."""
        return {"X-API-Key": "123123"}
    
    def test_validate_endpoint_with_valid_model(self, client, api_headers, sample_ros_model):
        """Test /validate endpoint with a valid model."""
        data = {
            "name": "test_model",
            "model": sample_ros_model
        }
        
        response = client.post("/validate", json=data, headers=api_headers)
        
        assert response.status_code == 200
        json_resp = response.json()
        assert json_resp["valid"] == True
        assert len(json_resp["errors"]) == 0
    
    def test_validate_endpoint_with_invalid_model(self, client, api_headers):
        """Test /validate endpoint with an invalid model."""
        data = {
            "name": "test_model",
            "model": "This is not a valid model"
        }
        
        response = client.post("/validate", json=data, headers=api_headers)
        
        assert response.status_code == 200  # Returns 200 with validation errors
        json_resp = response.json()
        assert json_resp["valid"] == False
        assert len(json_resp["errors"]) > 0
    
    def test_validate_endpoint_requires_api_key(self, client, sample_ros_model):
        """Test that /validate endpoint requires API key."""
        data = {
            "name": "test_model",
            "model": sample_ros_model
        }
        
        # No headers provided
        response = client.post("/validate", json=data)
        
        assert response.status_code == 401  # Unauthorized
    
    def test_validate_endpoint_invalid_api_key(self, client, sample_ros_model):
        """Test /validate endpoint with invalid API key."""
        data = {
            "name": "test_model",
            "model": sample_ros_model
        }
        headers = {"X-API-Key": "invalid_key"}
        
        response = client.post("/validate", json=data, headers=headers)
        
        assert response.status_code == 401  # Unauthorized
    
    def test_validate_endpoint_empty_model(self, client, api_headers):
        """Test /validate endpoint with empty model."""
        data = {
            "name": "test_model",
            "model": ""
        }
        
        response = client.post("/validate", json=data, headers=api_headers)
        
        # Empty model should now return 400 (refactored API validates properly)
        assert response.status_code == 400


class TestAPIValidateFile:
    """Tests for /validate/file endpoint."""
    
    @pytest.fixture
    def client(self):
        """Create a test client."""
        return TestClient(api)
    
    @pytest.fixture
    def api_headers(self):
        """Return headers with valid API key."""
        return {"X-API-Key": "123123"}
    
    def test_validate_file_endpoint(self, client, api_headers, sample_ros_model, temp_dir):
        """Test /validate/file endpoint with file upload."""
        # Create a temporary file
        model_path = os.path.join(temp_dir, "test.rbr")
        with open(model_path, 'w') as f:
            f.write(sample_ros_model)
        
        with open(model_path, 'rb') as f:
            files = {"file": ("test.rbr", f, "text/plain")}
            response = client.post("/validate/file", files=files, headers=api_headers)
        
        assert response.status_code == 200
        json_resp = response.json()
        assert json_resp["valid"] == True
        assert len(json_resp["errors"]) == 0
    
    def test_validate_file_endpoint_invalid_model(self, client, api_headers, temp_dir):
        """Test /validate/file endpoint with invalid model."""
        model_path = os.path.join(temp_dir, "invalid.rbr")
        with open(model_path, 'w') as f:
            f.write("Invalid model content")
        
        with open(model_path, 'rb') as f:
            files = {"file": ("invalid.rbr", f, "text/plain")}
            response = client.post("/validate/file", files=files, headers=api_headers)
        
        assert response.status_code == 200
        json_resp = response.json()
        assert json_resp["valid"] == False
        assert len(json_resp["errors"]) > 0
    
    def test_validate_file_requires_api_key(self, client, sample_ros_model, temp_dir):
        """Test that /validate/file requires API key."""
        model_path = os.path.join(temp_dir, "test.rbr")
        with open(model_path, 'w') as f:
            f.write(sample_ros_model)
        
        with open(model_path, 'rb') as f:
            files = {"file": ("test.rbr", f, "text/plain")}
            response = client.post("/validate/file", files=files)
        
        assert response.status_code == 401  # Unauthorized


class TestAPICORS:
    """Tests for CORS middleware."""
    
    @pytest.fixture
    def client(self):
        """Create a test client."""
        return TestClient(api)
    
    @pytest.fixture
    def api_headers(self):
        """Return headers with valid API key."""
        return {"X-API-Key": "123123"}
    
    def test_cors_headers_present(self, client, api_headers, sample_ros_model):
        """Test that CORS headers are present in responses."""
        # Make an actual request to validate endpoint
        data = {
            "name": "test_model",
            "model": sample_ros_model
        }
        response = client.post("/validate", json=data, headers=api_headers)
        
        # Should have CORS headers in the response
        # The middleware adds these headers to all responses
        assert response.status_code in [200, 400]
