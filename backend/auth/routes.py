"""
Simplified Authentication routes using Supabase
"""

from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, EmailStr, Field
from typing import List, Optional
from enum import Enum
from config.supabase_client import supabase as supabase_client

router = APIRouter(prefix="/api/auth", tags=["Authentication"])


# Enums for validation
class ProgrammingKnowledge(str, Enum):
    beginner = "beginner"
    intermediate = "intermediate"
    advanced = "advanced"


class LearningStyle(str, Enum):
    hands_on = "hands-on"
    theory_first = "theory-first"
    mixed = "mixed"


# Pydantic models
class UserProfileCreate(BaseModel):
    programming_knowledge: ProgrammingKnowledge = Field(
        ..., 
        description="Programming experience level",
        example="beginner"
    )
    prior_robotics_experience: bool = Field(
        default=False,
        description="Whether user has robotics experience"
    )
    learning_goals: List[str] = Field(
        default_factory=list,
        description="User's learning goals (ros2, simulation, ai, etc.)",
        example=["ros2", "simulation"]
    )
    preferred_learning_style: LearningStyle = Field(
        ...,
        description="Learning style preference",
        example="hands-on"
    )


class SignupRequest(BaseModel):
    email: EmailStr = Field(..., example="user@example.com")
    password: str = Field(..., min_length=8, example="SecurePass123")
    name: str = Field(..., example="John Doe")
    profile: UserProfileCreate


class SigninRequest(BaseModel):
    email: EmailStr
    password: str


@router.post("/signup")
async def signup(data: SignupRequest):
    """
    Register new user using Supabase Auth
    
    Supabase automatically:
    - Hashes password
    - Creates user in auth.users table
    - Returns session token
    
    Note: Email confirmation may be required depending on Supabase settings.
    To disable: Supabase Dashboard > Authentication > Providers > Email > Disable "Confirm email"
    """
    try:
        # Sign up user with Supabase Auth
        auth_response = supabase_client.auth.sign_up({
            "email": data.email,
            "password": data.password,
            "options": {
                "data": {
                    "name": data.name,
                    "display_name": data.name
                }
            }
        })
        
        if not auth_response.user:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Failed to create user"
            )
        
        # Update user profile with Enum values
        profile_response = supabase_client.table('user_profiles').update({
            "programming_knowledge": data.profile.programming_knowledge.value,
            "prior_robotics_experience": data.profile.prior_robotics_experience,
            "learning_goals": data.profile.learning_goals,
            "preferred_learning_style": data.profile.preferred_learning_style.value
        }).eq('id', auth_response.user.id).execute()
        
        # Check if email confirmation is required
        needs_confirmation = auth_response.user.email_confirmed_at is None
        
        return {
            "user": auth_response.user,
            "session": auth_response.session,
            "message": "User created successfully" if not needs_confirmation else "User created! Please check your email to confirm your account before signing in.",
            "email_confirmation_required": needs_confirmation
        }
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Signup error: {str(e)}"
        )


@router.post("/signin")
async def signin(data: SigninRequest):
    """
    Login user using Supabase Auth
    
    Returns access token and user data
    """
    try:
        # Sign in with Supabase Auth
        auth_response = supabase_client.auth.sign_in_with_password({
            "email": data.email,
            "password": data.password
        })
        
        if not auth_response.user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid email or password"
            )
        
        # Get user profile
        profile = supabase_client.table('user_profiles')\
            .select('*')\
            .eq('id', auth_response.user.id)\
            .single()\
            .execute()
        
        return {
            "access_token": auth_response.session.access_token,
            "token_type": "bearer",
            "user": auth_response.user,
            "profile": profile.data if profile.data else None
        }
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password"
        )


@router.post("/signout")
async def signout():
    """
    Logout user
    """
    try:
        supabase_client.auth.sign_out()
        return {"message": "Successfully signed out"}
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Signout error: {str(e)}"
        )


@router.get("/user")
async def get_current_user(token: str):
    """
    Get current user from token
    """
    try:
        user = supabase_client.auth.get_user(token)
        
        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid token"
            )
        
        # Get profile
        profile = supabase_client.table('user_profiles')\
            .select('*')\
            .eq('id', user.user.id)\
            .single()\
            .execute()
        
        return {
            "user": user.user,
            "profile": profile.data if profile.data else None
        }
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token"
        )


@router.put("/profile")
async def update_profile(token: str, profile_data: UserProfileCreate):
    """
    Update user profile
    
    Requires authentication token
    """
    try:
        # Verify token and get user
        user = supabase_client.auth.get_user(token)
        
        if not user or not user.user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid token"
            )
        
        # Update profile in database
        profile_response = supabase_client.table('user_profiles').update({
            "programming_knowledge": profile_data.programming_knowledge.value,
            "prior_robotics_experience": profile_data.prior_robotics_experience,
            "learning_goals": profile_data.learning_goals,
            "preferred_learning_style": profile_data.preferred_learning_style.value
        }).eq('id', user.user.id).execute()
        
        if not profile_response.data:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Profile not found"
            )
        
        return {
            "message": "Profile updated successfully",
            "profile": profile_response.data[0]
        }
    
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Profile update error: {str(e)}"
        )
