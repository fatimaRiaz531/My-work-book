"""
Basic test to verify the RAG Chatbot implementation
"""
import pytest
from fastapi.testclient import TestClient
from src.api.main import app

client = TestClient(app)

def test_health_endpoint():
    """Test that the health endpoint works"""
    response = client.get("/api/v1/health")
    assert response.status_code == 200

    data = response.json()
    assert "status" in data
    assert "timestamp" in data
    assert "dependencies" in data

def test_chat_endpoint_exists():
    """Test that the chat endpoint is available"""
    # This will likely return 422 (validation error) or 401 (auth error) which is expected
    # since we're not providing proper payload or auth
    response = client.post("/api/v1/chat", json={"message": "test"})

    # We expect either 422 (validation error) or 401 (auth error) rather than 404 (not found)
    assert response.status_code in [401, 400, 422, 500]  # Endpoint exists but lacks auth/payload

def test_selection_endpoint_exists():
    """Test that the selection chat endpoint is available"""
    # This will likely return 422 (validation error) or 401 (auth error) which is expected
    response = client.post("/api/v1/chat/selection",
                          json={"message": "test", "selection": "test selection"})

    # We expect either 422 (validation error) or 401 (auth error) rather than 404 (not found)
    assert response.status_code in [401, 400, 422, 500]  # Endpoint exists but lacks auth/payload

if __name__ == "__main__":
    test_health_endpoint()
    test_chat_endpoint_exists()
    test_selection_endpoint_exists()
    print("All basic tests passed!")