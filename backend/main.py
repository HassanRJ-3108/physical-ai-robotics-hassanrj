"""
FastAPI backend for RAG Chatbot
Connects the OpenAI Agents SDK chatbot with REST API endpoints
"""

import os
import uuid
from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import logging
from dotenv import load_dotenv

# Load environment variables FIRST
load_dotenv()

from chatbot import BookChatbot
from models.schemas import ChatRequest, ChatResponse, HealthResponse, ErrorResponse
from auth.routes import router as auth_router
from auth.middleware import get_user_profile_optional

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Global chatbot instance
chatbot_instance = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifespan context manager for FastAPI"""
    global chatbot_instance
    
    # Startup
    logger.info("🚀 Starting up...")
    
    # Initialize chatbot
    try:
        logger.info("🤖 Initializing chatbot...")
        chatbot_instance = BookChatbot()
        logger.info("✅ Chatbot initialized successfully!")
    except Exception as e:
        logger.error(f"❌ Failed to initialize chatbot: {e}")
        chatbot_instance = None
    
    yield
    
    # Shutdown
    logger.info("👋 Shutting down...")


# Create FastAPI app
app = FastAPI(
    title="Physical AI Book Chatbot API",
    description="RAG chatbot API for Physical AI & Robotics book using OpenAI Agents SDK and Qdrant",
    version="1.0.0",
    lifespan=lifespan
)

# CORS middleware - allow Docusaurus frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Docusaurus dev
        "http://localhost:3001",
        "http://127.0.0.1:3000",
        "*"  # Allow all for development (restrict in production)
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include authentication routes
app.include_router(auth_router)


@app.get("/", tags=["Root"])
async def root():
    """Root endpoint"""
    return {
        "message": "Physical AI Book Chatbot API",
        "docs": "/docs",
        "health": "/health"
    }


@app.get("/health", response_model=HealthResponse, tags=["Health"])
async def health_check():
    """Health check endpoint"""
    return HealthResponse(
        status="healthy" if chatbot_instance else "degraded",
        chatbot_ready=chatbot_instance is not None,
        version="1.0.0"
    )


@app.post("/api/chat", response_model=ChatResponse, tags=["Chat"])
async def chat(
    request: ChatRequest,
    user_profile: dict = Depends(get_user_profile_optional)
):
    """
    Chat endpoint - sends message to RAG chatbot and returns response
    
    Works for both authenticated and guest users.
    Authenticated users get personalized responses based on their profile.
    
    Args:
        request: ChatRequest with user message and optional conversation history
        user_profile: Optional user profile (injected by auth middleware)
    
    Returns:
        ChatResponse with AI assistant's response
    """
    if not chatbot_instance:
        raise HTTPException(
            status_code=503,
            detail="Chatbot is not initialized. Please check server logs."
        )
    
    try:
        # Generate conversation ID if not provided
        conversation_id = request.conversation_id or str(uuid.uuid4())
        
        is_authenticated = user_profile is not None
        logger.info(f"💬 Processing chat request (conv_id: {conversation_id}, auth: {is_authenticated})")
        logger.info(f"   User: {request.message[:100]}...")
        
        # Call chatbot (will be personalized if user_profile is provided)
        result = await chatbot_instance.chat(request.message)
        
        if not result['success']:
            raise HTTPException(
                status_code=500,
                detail=result.get('error', 'Unknown error occurred')
            )
        
        logger.info(f"   ✅ Response generated successfully")
        
        
        return ChatResponse(
            response=result['response'],
            conversation_id=conversation_id,
            success=True
        )
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"❌ Error in chat endpoint: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Internal server error: {str(e)}"
        )


@app.post("/api/chat/reset", tags=["Chat"])
async def reset_chat(conversation_id: str = None):
    """
    Reset chat conversation
    
    Args:
        conversation_id: Optional conversation ID to reset
    
    Returns:
        Success message
    """
    # For now, just return success
    # In future, this will clear conversation history from database
    return {
        "message": "Chat reset successfully",
        "conversation_id": conversation_id or "all"
    }


if __name__ == "__main__":
    import uvicorn
    
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )
