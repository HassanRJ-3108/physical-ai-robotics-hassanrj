"""
User service for database operations
"""

from uuid import UUID
from typing import Optional
from datetime import datetime, timedelta
from database.connection import db
from models.user import UserCreate, UserProfileUpdate
from auth.utils import hash_password, generate_session_id
import asyncpg
import json


class UserService:
    """Service for user-related database operations"""
    
    async def create_user(self, user_data: UserCreate) -> dict:
        """
        Create new user with profile
        
        Args:
            user_data: User creation data
        
        Returns:
            Created user data
        """
        # Hash password
        password_hash = hash_password(user_data.password)
        
        async with db.connection() as conn:
            # Start transaction
            async with conn.transaction():
                # Create user
                user = await conn.fetchrow(
                    """
                    INSERT INTO users (email, name, password_hash)
                    VALUES ($1, $2, $3)
                    RETURNING id, email, name, email_verified, created_at
                    """,
                    user_data.email,
                    user_data.name,
                    password_hash
                )
                
                # Create user profile
                profile = await conn.fetchrow(
                    """
                    INSERT INTO user_profiles (
                        user_id,
                        programming_knowledge,
                        prior_robotics_experience,
                        learning_goals,
                        preferred_learning_style,
                        interests
                    )
                    VALUES ($1, $2, $3, $4, $5, $6)
                    RETURNING id
                    """,
                    user['id'],
                    user_data.profile.programming_knowledge,
                    user_data.profile.prior_robotics_experience,
                    user_data.profile.learning_goals,
                    user_data.profile.preferred_learning_style,
                    json.dumps({})  # Default empty interests
                )
                
                return dict(user)
    
    async def get_user_by_email(self, email: str) -> Optional[dict]:
        """Get user by email"""
        user = await db.fetchrow(
            """
            SELECT id, email, name, password_hash, email_verified, created_at
            FROM users
            WHERE email = $1
            """,
            email
        )
        return dict(user) if user else None
    
    async def get_user_by_id(self, user_id: UUID) -> Optional[dict]:
        """Get user by ID"""
        user = await db.fetchrow(
            """
            SELECT id, email, name, email_verified, created_at
            FROM users
            WHERE id = $1
            """,
            user_id
        )
        return dict(user) if user else None
    
    async def get_user_profile(self, user_id: UUID) -> Optional[dict]:
        """Get complete user profile"""
        profile = await db.fetchrow(
            """
            SELECT 
                u.id,
                u.email,
                u.name,
                u.email_verified,
                p.programming_knowledge,
                p.prior_robotics_experience,
                p.learning_goals,
                p.preferred_learning_style,
                p.current_chapter,
                p.completed_chapters,
                p.total_questions_asked,
                p.interests
            FROM users u
            LEFT JOIN user_profiles p ON u.id = p.user_id
            WHERE u.id = $1
            """,
            user_id
        )
        return dict(profile) if profile else None
    
    async def update_profile(self, user_id: UUID, profile_data: UserProfileUpdate) -> dict:
        """Update user profile"""
        # Build dynamic update query
        updates = []
        values = []
        param_num = 1
        
        if profile_data.programming_knowledge is not None:
            updates.append(f"programming_knowledge = ${param_num}")
            values.append(profile_data.programming_knowledge)
            param_num += 1
        
        if profile_data.prior_robotics_experience is not None:
            updates.append(f"prior_robotics_experience = ${param_num}")
            values.append(profile_data.prior_robotics_experience)
            param_num += 1
        
        if profile_data.learning_goals is not None:
            updates.append(f"learning_goals = ${param_num}")
            values.append(profile_data.learning_goals)
            param_num += 1
        
        if profile_data.preferred_learning_style is not None:
            updates.append(f"preferred_learning_style = ${param_num}")
            values.append(profile_data.preferred_learning_style)
            param_num += 1
        
        if profile_data.current_chapter is not None:
            updates.append(f"current_chapter = ${param_num}")
            values.append(profile_data.current_chapter)
            param_num += 1
        
        if profile_data.interests is not None:
            updates.append(f"interests = ${param_num}")
            values.append(profile_data.interests)
            param_num += 1
        
        if not updates:
            return await self.get_user_profile(user_id)
        
        values.append(user_id)
        query = f"""
            UPDATE user_profiles
            SET {", ".join(updates)}
            WHERE user_id = ${param_num}
            RETURNING *
        """
        
        result = await db.fetchrow(query, *values)
        return dict(result) if result else None
    
    async def increment_questions_asked(self, user_id: UUID):
        """Increment total questions asked counter"""
        await db.execute(
            """
            UPDATE user_profiles
            SET total_questions_asked = total_questions_asked + 1
            WHERE user_id = $1
            """,
            user_id
        )
    
    async def update_current_chapter(self, user_id: UUID, chapter: str):
        """Update user's current chapter"""
        await db.execute(
            """
            UPDATE user_profiles
            SET current_chapter = $1
            WHERE user_id = $2
            """,
            chapter,
            user_id
        )
    
    async def create_session(self, user_id: UUID, ip_address: str = None, user_agent: str = None) -> dict:
        """Create new session for user"""
        session_id = generate_session_id()
        expires_at = datetime.utcnow() + timedelta(days=7)
        
        await db.execute(
            """
            INSERT INTO sessions (id, user_id, expires_at, ip_address, user_agent)
            VALUES ($1, $2, $3, $4, $5)
            """,
            session_id,
            user_id,
            expires_at,
            ip_address,
            user_agent
        )
        
        return {
            "session_id": session_id,
            "user_id": user_id,
            "expires_at": expires_at
        }
    
    async def get_session(self, session_id: str) -> Optional[dict]:
        """Get session by ID"""
        session = await db.fetchrow(
            """
            SELECT s.id, s.user_id, s.expires_at, u.email, u.name
            FROM sessions s
            JOIN users u ON s.user_id = u.id
            WHERE s.id = $1 AND s.expires_at > NOW()
            """,
            session_id
        )
        return dict(session) if session else None
    
    async def delete_session(self, session_id: str):
        """Delete session (logout)"""
        await db.execute(
            """
            DELETE FROM sessions WHERE id = $1
            """,
            session_id
        )
    
    async def delete_all_user_sessions(self, user_id: UUID):
        """Delete all sessions for a user"""
        await db.execute(
            """
            DELETE FROM sessions WHERE user_id = $1
            """,
            user_id
        )


# Global instance
user_service = UserService()
