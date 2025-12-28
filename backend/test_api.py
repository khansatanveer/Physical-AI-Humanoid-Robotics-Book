"""
Test script to validate the FastAPI RAG integration
"""
import requests
import json

def test_api():
    # Test the health endpoint
    print("Testing health endpoint...")
    health_response = requests.get("http://localhost:8000/health")
    print(f"Health endpoint status: {health_response.status_code}")
    print(f"Health endpoint response: {health_response.json()}")

    # Test the chat endpoint
    print("\nTesting chat endpoint...")
    chat_payload = {
        "query": "What are the key concepts in humanoid robotics?"
    }

    headers = {
        "Content-Type": "application/json"
    }

    try:
        chat_response = requests.post("http://localhost:8000/chat", json=chat_payload, headers=headers)
        print(f"Chat endpoint status: {chat_response.status_code}")

        if chat_response.status_code == 200:
            response_data = chat_response.json()
            print("Chat endpoint response:")
            print(f"  Response preview: {response_data['response'][:100]}...")
            print(f"  Source chunks: {len(response_data['source_chunks'])} chunks")
            print(f"  Grounded: {response_data['grounded']}")
            print(f"  Confidence: {response_data['confidence']}")
            print(f"  Query ID: {response_data['query_id']}")
            print("✅ API test passed!")
        else:
            print(f"❌ Chat endpoint failed with status {chat_response.status_code}")
            print(f"Response: {chat_response.text}")
    except Exception as e:
        print(f"❌ Error testing API: {e}")
        print("Note: Make sure the API server is running with 'python api.py'")

if __name__ == "__main__":
    test_api()