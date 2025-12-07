"""
User and UserProfile Pydantic models
"""

from pydantic import BaseModel, EmailStr, Field, validator
from typing import Optional, List
from datetime import datetime
from uuid import UUID


class UserProfileCreate(BaseModel):
    """Profile data for signup"""
    programming_knowledge: str = Field(
        ...,
        description="Programming experience level",
        pattern="^(beginner|intermediate|advanced)$"
    )
    prior_robotics_experience: bool = Field(
        default=False,
        description="Whether user has robotics experience"
    )
    learning_goals: List[str] = Field(
        default_factory=list,
        description="User's learning goals (ros2, simulation, ai, etc.)"
    )
    preferred_learning_style: str = Field(
        ...,
        description="Learning style preference",
        pattern="^(hands-on|theory-first|mixed)$"
    )


class UserProfile(UserProfileCreate):
    """Complete user profile"""
    current_chapter: Optional[str] = Field(
        None,
        description="Current chapter being studied"
    )
    interests: Optional[dict] = Field(
        default_factory=dict,
        description="Additional interests and preferences"
    )


class UserCreate(BaseModel):
    """Model for creating a new user"""
    email: EmailStr
    password: str = Field(..., min_length=8)
    name: str = Field(..., min_length=2, max_length=100)
    profile: UserProfileCreate
    
    @validator('password')
    def validate_password(cls, v):
        if len(v) < 8:
            raise ValueError('Password must be at least 8 characters')
        if not any(c.isupper() for c in v):
            raise ValueError('Password must contain at least one uppercase letter')
        if not any(c.islower() for c in v):
            raise ValueError('Password must contain at least one lowercase letter')
        if not any(c.isdigit() for c in v):
            raise ValueError('Password must contain at least one digit')
        return v


class UserLogin(BaseModel):
    """Model for user login"""
    email: EmailStr
    password: str


class UserResponse(BaseModel):
    """User response model (without password)"""
    id: UUID
    email: str
    name: str
    email_verified: bool
    created_at: datetime
    
    class Config:
        from_attributes = True


class UserProfileResponse(BaseModel):
    """User profile response"""
    id: UUID
    email: str
    name: str
    email_verified: bool
    programming_knowledge: Optional[str]
    prior_robotics_experience: Optional[bool]
    learning_goals: List[str] = []
    preferred_learning_style: Optional[str]
    current_chapter: Optional[str]
    total_questions_asked: int = 0
    interests: dict = {}
    
    class Config:
        from_attributes = True


class UserProfileUpdate(BaseModel):
    """Update user profile"""
    programming_knowledge: Optional[str] = Field(
        None,
        pattern="^(beginner|intermediate|advanced)$"
    )
    prior_robotics_experience: Optional[bool] = None
    learning_goals: Optional[List[str]] = None
    preferred_learning_style: Optional[str] = Field(
        None,
        pattern="^(hands-on|theory-first|mixed)$"
    )
    current_chapter: Optional[str] = None
    interests: Optional[dict] = None


class SessionResponse(BaseModel):
    """Session response"""
    session_id: str
    user: UserResponse
    expires_at: datetime


class TokenResponse(BaseModel):
    """Authentication token response"""
    access_token: str
    token_type: str = "bearer"
    expires_in: int  # seconds
    user: UserResponse
