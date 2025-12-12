#!/usr/bin/env python
"""
Quick test to verify chatbot setup without API keys
Tests: imports, server connectivity, health check
"""

import time
import subprocess
import requests
import sys

print("🧪 Testing Chatbot Setup\n")

# Test 1: Imports
print("1️⃣  Testing imports...")
try:
    from qdrant_client import QdrantClient
    from openai import OpenAI
    from fastapi import FastAPI
    print("   ✅ All imports successful\n")
except ImportError as e:
    print(f"   ❌ Import failed: {e}\n")
    sys.exit(1)

# Test 2: Check if server is running
print("2️⃣  Checking server status...")
try:
    # Give server a moment to start
    time.sleep(1)
    response = requests.get("http://localhost:8000/", timeout=2)
    print(f"   ✅ Server is running (port 8000)\n")
    print(f"   Response: {response.json()}\n")
except requests.exceptions.ConnectionError:
    print("   ⚠️  Server not running yet")
    print("   📍 Start server with: python main.py\n")
except Exception as e:
    print(f"   ⚠️  Could not connect: {e}\n")

# Test 3: Health check (will show missing API keys)
print("3️⃣  Testing health endpoint...")
try:
    response = requests.get("http://localhost:8000/health", timeout=2)
    health = response.json()
    print(f"   Status: {health['status']}")
    print(f"   Qdrant: {health['qdrant_connected']}")
    print(f"   OpenAI: {health['openai_connected']}")
    print()
except Exception as e:
    print(f"   Could not reach health endpoint: {e}\n")

# Test 4: Show what's needed
print("4️⃣  What you need to do:")
print("   📌 Get Qdrant API key: https://cloud.qdrant.io/")
print("   📌 Get OpenAI API key: https://platform.openai.com/api-keys")
print("   📌 Update chatbot/.env with your keys")
print("   📌 Run: python index_content.py (to index textbook)")
print("   📌 Test: python query_test.py (after indexing)\n")

print("✅ Setup verification complete!")
