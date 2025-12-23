from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel
from typing import Optional
import hashlib
import secrets
from datetime import datetime, timedelta
from ..models.user import User as UserModel
from ..config.database import get_db
from sqlalchemy.orm import Session
import jwt
import logging

logger = logging.getLogger(__name__)

router = APIRouter()

# Pydantic models for auth requests/responses
class SignupRequest(BaseModel):
    email: str
    password: str
    name: str
    softwareBackground: Optional[str] = None
    hardwareBackground: Optional[str] = None

class SigninRequest(BaseModel):
    email: str
    password: str

class UpdateProfileRequest(BaseModel):
    softwareBackground: Optional[str] = None
    hardwareBackground: Optional[str] = None

class AuthResponse(BaseModel):
    success: bool
    user: Optional[dict] = None
    message: Optional[str] = None

# Secret key for JWT (in production, use environment variable)
SECRET_KEY = "your-secret-key-change-in-production"  # Should be in env
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30 * 24 * 60  # 30 days

def hash_password(password: str) -> str:
    """Hash password using SHA256 with salt"""
    salt = secrets.token_hex(16)
    hashed = hashlib.sha256((password + salt).encode()).hexdigest()
    return f"{hashed}:{salt}"

def verify_password(password: str, hashed: str) -> bool:
    """Verify password against hash"""
    stored_hash, salt = hashed.split(":")
    password_hash = hashlib.sha256((password + salt).encode()).hexdigest()
    return password_hash == stored_hash

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

@router.post("/auth/signup", response_model=AuthResponse)
async def signup(request: SignupRequest, db: Session = Depends(get_db)):
    """User signup endpoint"""
    try:
        # Check if user already exists
        existing_user = db.query(UserModel).filter(UserModel.email == request.email).first()
        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="User with this email already exists"
            )

        # Hash password
        hashed_password = hash_password(request.password)

        # Create new user
        user = UserModel(
            email=request.email,
            password_hash=hashed_password,
            name=request.name,
            software_background=request.softwareBackground,
            hardware_background=request.hardwareBackground
        )

        db.add(user)
        db.commit()
        db.refresh(user)

        # Create JWT token
        access_token = create_access_token(
            data={"sub": user.email, "user_id": str(user.id)},
            expires_delta=timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
        )

        # Return user data with token
        user_data = {
            "id": str(user.id),
            "email": user.email,
            "name": user.name,
            "softwareBackground": user.software_background,
            "hardwareBackground": user.hardware_background,
            "token": access_token
        }

        logger.info(f"User registered successfully: {user.email}")
        return AuthResponse(success=True, user=user_data, message="User created successfully")

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in signup: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error during registration"
        )

@router.post("/auth/signin", response_model=AuthResponse)
async def signin(request: SigninRequest, db: Session = Depends(get_db)):
    """User signin endpoint"""
    try:
        # Find user by email
        user = db.query(UserModel).filter(UserModel.email == request.email).first()
        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid email or password"
            )

        # Verify password
        if not verify_password(request.password, user.password_hash):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid email or password"
            )

        # Create JWT token
        access_token = create_access_token(
            data={"sub": user.email, "user_id": str(user.id)},
            expires_delta=timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
        )

        # Return user data with token
        user_data = {
            "id": str(user.id),
            "email": user.email,
            "name": user.name,
            "softwareBackground": user.software_background,
            "hardwareBackground": user.hardware_background,
            "token": access_token
        }

        logger.info(f"User signed in successfully: {user.email}")
        return AuthResponse(success=True, user=user_data, message="Sign in successful")

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in signin: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error during sign in"
        )

@router.get("/auth/user", response_model=AuthResponse)
async def get_user(token: str, db: Session = Depends(get_db)):
    """Get current user info"""
    try:
        # Decode JWT token
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        user_id = payload.get("user_id")

        if not user_id:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid token"
            )

        # Get user from database
        user = db.query(UserModel).filter(UserModel.id == user_id).first()
        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="User not found"
            )

        # Return user data
        user_data = {
            "id": str(user.id),
            "email": user.email,
            "name": user.name,
            "softwareBackground": user.software_background,
            "hardwareBackground": user.hardware_background
        }

        return AuthResponse(success=True, user=user_data)

    except jwt.PyJWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token"
        )
    except Exception as e:
        logger.error(f"Error getting user: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )

@router.put("/auth/profile", response_model=AuthResponse)
async def update_profile(request: UpdateProfileRequest, token: str, db: Session = Depends(get_db)):
    """Update user profile"""
    try:
        # Decode JWT token
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        user_id = payload.get("user_id")

        if not user_id:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid token"
            )

        # Get user from database
        user = db.query(UserModel).filter(UserModel.id == user_id).first()
        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="User not found"
            )

        # Update user profile
        if request.softwareBackground is not None:
            user.software_background = request.softwareBackground
        if request.hardwareBackground is not None:
            user.hardware_background = request.hardwareBackground

        db.commit()
        db.refresh(user)

        # Return updated user data
        user_data = {
            "id": str(user.id),
            "email": user.email,
            "name": user.name,
            "softwareBackground": user.software_background,
            "hardwareBackground": user.hardware_background
        }

        logger.info(f"User profile updated: {user.email}")
        return AuthResponse(success=True, user=user_data, message="Profile updated successfully")

    except jwt.PyJWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token"
        )
    except Exception as e:
        logger.error(f"Error updating profile: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )

@router.post("/auth/signout", response_model=AuthResponse)
async def signout():
    """Sign out user (client-side token removal)"""
    return AuthResponse(success=True, message="Signed out successfully")