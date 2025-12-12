#!/usr/bin/env python
"""
Setup verification script - Test all connections
"""

import os
import sys
from pathlib import Path

# Add parent dir to path
sys.path.insert(0, str(Path(__file__).parent))

print("\n" + "="*70)
print("🧪 CHATBOT SETUP VERIFICATION")
print("="*70 + "\n")

# Load .env
from dotenv import load_dotenv
load_dotenv()

# Test 1: Qdrant Connection
print("1️⃣  Testing Qdrant Connection...")
try:
    from qdrant_client import QdrantClient
    
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_key = os.getenv("QDRANT_API_KEY")
    
    if not qdrant_url or not qdrant_key:
        print("   ⚠️  Missing Qdrant credentials in .env")
        print("      • QDRANT_URL:", "✅" if qdrant_url else "❌")
        print("      • QDRANT_API_KEY:", "✅" if qdrant_key else "❌")
    else:
        client = QdrantClient(url=qdrant_url, api_key=qdrant_key)
        collections = client.get_collections()
        print(f"   ✅ Qdrant connected!")
        print(f"   📚 Collections: {len(collections.collections)} existing")
        
except Exception as e:
    print(f"   ❌ Qdrant error: {e}")

# Test 2: OpenAI Connection
print("\n2️⃣  Testing OpenAI Connection...")
try:
    from openai import OpenAI
    
    openai_key = os.getenv("OPENAI_API_KEY")
    
    if not openai_key:
        print("   ❌ Missing OPENAI_API_KEY in .env")
        print("   📍 Get key from: https://platform.openai.com/api-keys")
    elif not openai_key.startswith("sk-"):
        print(f"   ❌ Invalid key format (should start with 'sk-')")
    else:
        client = OpenAI(api_key=openai_key)
        models = client.models.list()
        print(f"   ✅ OpenAI connected!")
        print(f"   🤖 Available models: {len(models.data)}")
        
except Exception as e:
    print(f"   ❌ OpenAI error: {e}")

# Test 3: FastAPI Server
print("\n3️⃣  Testing FastAPI Server...")
try:
    import requests
    
    try:
        response = requests.get("http://localhost:8000/", timeout=2)
        print(f"   ✅ Server running on port 8000")
        print(f"   📊 Response: {response.status_code}")
    except requests.exceptions.ConnectionError:
        print(f"   ⚠️  Server not running")
        print(f"   📍 Start with: python main.py")
        
except Exception as e:
    print(f"   ⚠️  Connection test failed: {e}")

# Test 4: Dependencies
print("\n4️⃣  Testing Dependencies...")
try:
    import fastapi
    import uvicorn
    import pydantic
    import qdrant_client
    import openai
    
    print(f"   ✅ FastAPI {fastapi.__version__}")
    print(f"   ✅ Uvicorn {uvicorn.__version__}")
    print(f"   ✅ Pydantic {pydantic.__version__}")
    print(f"   ✅ Qdrant-client {qdrant_client.__version__}")
    print(f"   ✅ OpenAI {openai.__version__}")
    
except ImportError as e:
    print(f"   ❌ Missing package: {e}")

print("\n" + "="*70)
print("📋 SETUP SUMMARY")
print("="*70 + "\n")

# Check what's ready
qdrant_ok = bool(os.getenv("QDRANT_URL") and os.getenv("QDRANT_API_KEY"))
openai_ok = bool(os.getenv("OPENAI_API_KEY") and os.getenv("OPENAI_API_KEY").startswith("sk-"))

print(f"Qdrant:  {'✅ READY' if qdrant_ok else '❌ NEEDS SETUP'}")
print(f"OpenAI:  {'✅ READY' if openai_ok else '❌ NEEDS SETUP'}")
print(f"Server:  Check with: curl http://localhost:8000/")

if qdrant_ok and openai_ok:
    print("\n✅ ALL SYSTEMS GO! Ready to:")
    print("   1. python index_content.py  (index textbook)")
    print("   2. python main.py            (start server)")
    print("   3. curl http://localhost:8000/query (test)")
else:
    print("\n⏳ MISSING SETUP:")
    if not openai_ok:
        print("   • Get OpenAI key: https://platform.openai.com/api-keys")
        print("   • Add to .env: OPENAI_API_KEY=sk-...")

print("\n" + "="*70 + "\n")
