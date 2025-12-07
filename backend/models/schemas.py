"""
Pydantic schemas for FastAPI requests and responses
"""

from pydantic import BaseModel, Field
from typing import List, Optional


class ChatMessage(BaseModel):
    """Single chat message"""
    role: str = Field(..., description="Message role: 'user' or 'assistant'")
    content: str = Field(..., description="Message content")


class ChatRequest(BaseModel):
    """Request model for chat endpoint"""
    message: str = Field(..., description="User's message", min_length=1)
    conversation_id: Optional[str] = Field(None, description="Conversation ID for session management")
    history: Optional[List[ChatMessage]] = Field(default_factory=list, description="Conversation history")


class ChatResponse(BaseModel):
    """Response model for chat endpoint"""
    response: str = Field(..., description="AI assistant's response")
    conversation_id: str = Field(..., description="Conversation ID")
    success: bool = Field(True, description="Whether request was successful")


class HealthResponse(BaseModel):
    """Health check response"""
    status: str = Field(..., description="API status")
    chatbot_ready: bool = Field(..., description="Whether chatbot is initialized")
    version: str = Field(default="1.0.0", description="API version")


class ErrorResponse(BaseModel):
    """Error response model"""
    error: str = Field(..., description="Error message")
    detail: Optional[str] = Field(None, description="Detailed error information")
    success: bool = Field(False, description="Always false for errors")
