# Neon PostgreSQL Setup Guide

Quick guide to set up Neon database for the chatbot authentication system.

## Step 1: Create Neon Account

1. Go to https://neon.tech
2. Sign up for free account
3. Verify email

## Step 2: Create Database

1. Click "Create Project"
2. Choose region (us-east-2 recommended)
3. Give it a name: `physical-ai-chatbot`
4. Click "Create Project"

## Step 3: Get Connection String

After project creation, you'll see the connection details:

```
DATABASE_URL: postgresql://user:password@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require
```

Copy this entire URL!

## Step 4: Run Database Migration

1. Create `.env` file in `backend/` directory:
```bash
cd backend
cp env.example .env
```

2. Edit `.env` and add your connection string:
```bash
DATABASE_URL=postgresql://user:password@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require
GEMINI_API_KEY=your_existing_key
AUTH_SECRET=your_random_secret  # Generate with: openssl rand -base64 32
```

3. Run the schema SQL file:

**Option A: Using Neon SQL Editor (Recommended)**
1. Go to your Neon dashboard
2. Click "SQL Editor"
3. Copy contents of `backend/database/schema.sql`
4. Paste and run

**Option B: Using psql**
```bash
psql "postgresql://user:password@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require" -f database/schema.sql
```

##Step 5: Test Connection

```bash
cd backend
uv run python -c "from database.connection import test_connection; import asyncio; asyncio.run(test_connection())"
```

You should see: `✅ Database test query result: 2`

## Step 6: Install New Dependencies

```bash
cd backend
uv sync
```

This will install:
- asyncpg (PostgreSQL adapter)
- argon2-cffi (Password hashing)
- python-jose (JWT tokens)
- passlib (Password utilities)

## Step 7: Update FastAPI Server

The authentication system is now ready! Restart your FastAPI server:

```bash
uv run python main.py
```

## Troubleshooting

### Connection timeout
- Check your Neon project is active
- Verify connection string is correct
- Ensure `sslmode=require` is in the URL

### Module not found
- Run `uv sync` to install all dependencies
- Check you're in the `backend/` directory

### Permission denied on schema.sql
- Use Neon SQL Editor instead
- Or check psql is installed

## Next Steps

After database is set up:
1. Test signup endpoint: `POST /api/auth/signup`
2. Test signin endpoint: `POST /api/auth/signin`
3. Integrate with frontend

## Environment Variables Summary

Your `.env` should have:
```bash
# Gemini (existing)
GEMINI_API_KEY=AIzaSy...

# Neon Database (new)
DATABASE_URL=postgresql://...

# Authentication (new)
AUTH_SECRET=random_32_character_string
```

Generate AUTH_SECRET with:
```bash
openssl rand -base64 32
```

Done! 🎉
