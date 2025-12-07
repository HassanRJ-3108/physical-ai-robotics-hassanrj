"""
Simplified authentication middleware using Supabase
"""

from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from typing import Optional
from config.supabase_client import supabase as supabase_client

security = HTTPBearer()


async def get_user_profile_optional(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(HTTPBearer(auto_error=False))
) -> Optional[dict]:
    """
    Get user profile if authenticated, otherwise None
    
    This allows endpoints to work for both authenticated and guest users
    
    Args:
        credentials: Optional HTTP Bearer token
    
    Returns:
        User profile if authenticated, None otherwise
    """
    if not credentials:
        return None
    
    try:
        # Verify JWT and get user from Supabase
        user_response = supabase_client.auth.get_user(credentials.credentials)
        
        if not user_response or not user_response.user:
            return None
        
        # Get user profile from database
        profile_response = supabase_client.table('user_profiles')\
            .select('*')\
            .eq('id', user_response.user.id)\
            .single()\
            .execute()
        
        if not profile_response.data:
            return None
        
        # Combine user + profile data
        return {
            **user_response.user.model_dump(),
            **profile_response.data
        }
    
    except Exception as e:
        # Invalid token or other error - treat as guest
        return None


async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security)
) -> dict:
    """
    Get current authenticated user (required)
    
    Args:
        credentials: HTTP Bearer token
    
    Returns:
        User profile
    
    Raises:
        HTTPException: If token is invalid or user not found
    """
    try:
        # Verify JWT and get user from Supabase
        user_response = supabase_client.auth.get_user(credentials.credentials)
        
        if not user_response or not user_response.user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid or expired token",
                headers={"WWW-Authenticate": "Bearer"},
            )
        
        # Get user profile
        profile_response = supabase_client.table('user_profiles')\
            .select('*')\
            .eq('id', user_response.user.id)\
            .single()\
            .execute()
        
        if not profile_response.data:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User profile not found"
            )
        
        # Combine user + profile data
        return {
            **user_response.user.model_dump(),
            **profile_response.data
        }
    
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token",
            headers={"WWW-Authenticate": "Bearer"},
        )
