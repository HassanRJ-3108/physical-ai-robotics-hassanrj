"""
Test signup endpoint
"""
import requests
import json

url = "http://localhost:8000/api/auth/signup"
data = {
    "email": "testuser@example.com",
    "password": "TestPass123",
    "name": "Test User",
    "profile": {
        "programming_knowledge": "intermediate",
        "prior_robotics_experience": False,
        "learning_goals": ["ros2"],
        "preferred_learning_style": "hands-on"
    }
}

try:
    response = requests.post(url, json=data)
    print(f"Status Code: {response.status_code}")
    print(f"Response: {response.text}")
    if response.status_code == 200 or response.status_code == 201:
        print("✅ Signup successful!")
        print(json.dumps(response.json(), indent=2))
    else:
        print("❌ Signup failed!")
        try:
            print(json.dumps(response.json(), indent=2))
        except:
            print(response.text)
except Exception as e:
    print(f"❌ Error: {e}")
