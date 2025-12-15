# Serverless handler for AWS Lambda
import json
from backend.src.api.main import app
from mangum import Mangum

# Create the Mangum adapter for FastAPI
handler = Mangum(app)

def lambda_handler(event, context):
    # This function allows the app to work with AWS Lambda
    return handler(event, context)