"""
Database connection and utilities for Neon PostgreSQL
"""

import os
from dotenv import load_dotenv
import asyncpg
from typing import Optional
from contextlib import asynccontextmanager

# Load environment variables
load_dotenv()


class Database:
    """Database connection manager"""
    
    def __init__(self):
        self.database_url = os.getenv('DATABASE_URL')
        if not self.database_url:
            raise ValueError("DATABASE_URL not found in environment")
        self.pool: Optional[asyncpg.Pool] = None
    
    async def connect(self):
        """Create connection pool"""
        if not self.pool:
            self.pool = await asyncpg.create_pool(
                self.database_url,
                min_size=5,
                max_size=20,
                command_timeout=60
            )
            print("✅ Database connection pool created")
    
    async def disconnect(self):
        """Close connection pool"""
        if self.pool:
            await self.pool.close()
            print("👋 Database connection pool closed")
    
    @asynccontextmanager
    async def connection(self):
        """Get database connection from pool"""
        async with self.pool.acquire() as conn:
            yield conn
    
    async def execute(self, query: str, *args):
        """Execute a query"""
        async with self.connection() as conn:
            return await conn.execute(query, *args)
    
    async def fetch(self, query: str, *args):
        """Fetch multiple rows"""
        async with self.connection() as conn:
            return await conn.fetch(query, *args)
    
    async def fetchrow(self, query: str, *args):
        """Fetch single row"""
        async with self.connection() as conn:
            return await conn.fetchrow(query, *args)
    
    async def fetchval(self, query: str, *args):
        """Fetch single value"""
        async with self.connection() as conn:
            return await conn.fetchval(query, *args)


# Global database instance
db = Database()


async def get_db():
    """Dependency for FastAPI"""
    return db


async def test_connection():
    """Test database connection"""
    try:
        await db.connect()
        result = await db.fetchval("SELECT 1 + 1")
        print(f"✅ Database test query result: {result}")
        await db.disconnect()
        return True
    except Exception as e:
        print(f"❌ Database connection failed: {e}")
        return False


if __name__ == "__main__":
    import asyncio
    asyncio.run(test_connection())
