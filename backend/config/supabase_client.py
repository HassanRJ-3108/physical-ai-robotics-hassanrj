"""
Supabase client initialization
"""

import os
from dotenv import load_dotenv
from supabase import create_client, Client

# Load environment variables
load_dotenv()

# Supabase configuration
SUPABASE_URL = os.getenv("SUPABASE_URL")
SUPABASE_SERVICE_KEY = os.getenv("SUPABASE_SERVICE_ROLE_KEY")

if not SUPABASE_URL or not SUPABASE_SERVICE_KEY:
    raise ValueError("SUPABASE_URL and SUPABASE_SERVICE_ROLE_KEY must be set in environment")

# Create Supabase client
supabase: Client = create_client(SUPABASE_URL, SUPABASE_SERVICE_KEY)


async def test_connection():
    """Test Supabase connection"""
    try:
        # Simple query to test connection
        response = supabase.table('user_profiles').select("count", count='exact').execute()
        print(f"✅ Supabase connection successful!")
        print(f"   Profiles in database: {response.count}")
        return True
    except Exception as e:
        print(f"❌ Supabase connection failed: {e}")
        return False


if __name__ == "__main__":
    import asyncio
    asyncio.run(test_connection())
