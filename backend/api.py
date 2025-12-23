"""
Vercel-compatible FastAPI handler
This file serves as the entry point for Vercel deployment
"""
import sys
import os

# Add the backend directory to Python path to resolve imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)  # Adds project root to path

# Now we can import the main app
from backend.src.api.main import app

from mangum import Mangum

# Create the Mangum adapter for FastAPI
handler = Mangum(app, lifespan="off")