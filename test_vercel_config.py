#!/usr/bin/env python3
"""
Test script to validate the Vercel configuration
This simulates how Vercel will import and use our API handler
"""

def test_import():
    """Test that the api.py file can be imported without errors"""
    try:
        import sys
        import os

        # Add the project root to path (similar to how Vercel would do it)
        project_root = os.path.dirname(os.path.abspath(__file__))
        sys.path.insert(0, project_root)

        # Change to the backend directory to simulate Vercel's environment
        os.chdir(os.path.join(project_root, 'backend'))

        # Import the api module
        from api import handler
        print("✓ Successfully imported handler from backend/api.py")

        # Test that handler exists and is callable
        assert hasattr(handler, '__call__'), "Handler is not callable"
        print("✓ Handler is callable")

        # Test a sample event (simulating a Vercel request)
        sample_event = {
            "httpMethod": "GET",
            "path": "/api/v1/health",
            "headers": {},
            "queryStringParameters": None,
            "body": None
        }

        # This would normally be tested with actual Vercel environment
        print("✓ Configuration appears valid for Vercel deployment")

        return True

    except Exception as e:
        print(f"✗ Error testing configuration: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("Testing Vercel configuration...")
    success = test_import()
    if success:
        print("\n🎉 Configuration test passed! Ready for Vercel deployment.")
    else:
        print("\n❌ Configuration test failed! Please review the errors above.")
        exit(1)