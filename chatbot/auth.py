"""
Authentication module for user signup/signin with background questions
Uses JWT tokens and stores user profiles in database
"""

from fastapi import HTTPException, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel, EmailStr
from typing import Optional
from datetime import datetime, timedelta
from jose import JWTError, jwt
from passlib.context import CryptContext
import os
from dotenv import load_dotenv

load_dotenv()

# Security
SECRET_KEY = os.getenv("SECRET_KEY", "your-secret-key-change-in-production")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30 * 24 * 60  # 30 days

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
security = HTTPBearer()

# ============================================================================
# DATA MODELS
# ============================================================================

class UserSignup(BaseModel):
    """User signup with background questions"""
    email: EmailStr
    password: str
    name: str
    # Background questions
    software_background: str  # e.g., "Beginner", "Intermediate", "Advanced"
    hardware_background: str  # e.g., "None", "Basic", "Advanced"
    robotics_experience: str  # e.g., "None", "Some", "Expert"
    programming_languages: Optional[str] = None  # Comma-separated
    preferred_learning_style: Optional[str] = None  # e.g., "Visual", "Hands-on", "Reading"

class UserSignin(BaseModel):
    """User signin"""
    email: EmailStr
    password: str

class UserProfile(BaseModel):
    """User profile response"""
    id: str
    email: str
    name: str
    software_background: str
    hardware_background: str
    robotics_experience: str
    programming_languages: Optional[str] = None
    preferred_learning_style: Optional[str] = None
    created_at: str

class TokenResponse(BaseModel):
    """Auth token response"""
    access_token: str
    token_type: str = "bearer"
    user: UserProfile

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a password against hash"""
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password: str) -> str:
    """Hash a password"""
    return pwd_context.hash(password)

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    """Create JWT access token"""
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

def decode_token(token: str) -> dict:
    """Decode and verify JWT token"""
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        return payload
    except JWTError:
        raise HTTPException(status_code=401, detail="Invalid authentication credentials")

# ============================================================================
# DATABASE (Simple in-memory for demo - replace with Neon Postgres in production)
# ============================================================================

# In-memory user store (replace with Neon Postgres)
users_db: dict[str, dict] = {}

def get_user_by_email(email: str) -> Optional[dict]:
    """Get user by email"""
    return users_db.get(email.lower())

def create_user(user_data: UserSignup) -> dict:
    """Create new user"""
    email_lower = user_data.email.lower()
    if email_lower in users_db:
        raise HTTPException(status_code=400, detail="Email already registered")
    
    user_id = f"user_{len(users_db) + 1}"
    hashed_password = get_password_hash(user_data.password)
    
    user = {
        "id": user_id,
        "email": email_lower,
        "name": user_data.name,
        "hashed_password": hashed_password,
        "software_background": user_data.software_background,
        "hardware_background": user_data.hardware_background,
        "robotics_experience": user_data.robotics_experience,
        "programming_languages": user_data.programming_languages,
        "preferred_learning_style": user_data.preferred_learning_style,
        "created_at": datetime.utcnow().isoformat()
    }
    
    users_db[email_lower] = user
    return user

def get_current_user(credentials: HTTPAuthorizationCredentials = Depends(security)) -> dict:
    """Get current authenticated user from token"""
    token = credentials.credentials
    payload = decode_token(token)
    email: str = payload.get("sub")
    if email is None:
        raise HTTPException(status_code=401, detail="Invalid authentication credentials")
    
    user = get_user_by_email(email)
    if user is None:
        raise HTTPException(status_code=401, detail="User not found")
    
    return user

