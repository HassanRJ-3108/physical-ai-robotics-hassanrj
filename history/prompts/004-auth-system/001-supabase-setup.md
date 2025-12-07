# Prompt: Supabase Project Setup

**Date**: 2025-11-27  
**Agent**: Gemini CLI  
**Feature**: 004-auth-system

---

## Request

"Set up Supabase project for user authentication. Need database for user profiles with learning preferences."

---

## What Was Done

### 1. Created Supabase Project
- Project name: "Physical AI Book"
- Region: Southeast Asia
- Saved credentials to `.env`

### 2. Database Schema

Created `user_profiles` table:

```sql
CREATE TABLE user_profiles (
  id UUID PRIMARY KEY REFERENCES auth.users(id),
  programming_knowledge TEXT CHECK (programming_knowledge IN ('beginner', 'intermediate', 'advanced')),
  prior_robotics_experience BOOLEAN DEFAULT FALSE,
  learning_goals TEXT[],
  preferred_learning_style TEXT CHECK (preferred_learning_style IN ('hands-on', 'theory-first', 'mixed')),
  created_at TIMESTAMP DEFAULT NOW()
);
```

### 3. Row Level Security

```sql
ALTER TABLE user_profiles ENABLE ROW LEVEL SECURITY;

CREATE POLICY "Users read own profile" ON user_profiles
  FOR SELECT USING (auth.uid() = id);

CREATE POLICY "Users insert own profile" ON user_profiles
  FOR INSERT WITH CHECK (auth.uid() = id);
```

### 4. Auth Configuration
- Enabled email provider
- Enabled email confirmations
- Set redirect URLs

---

## Issues Fixed

**SSL Connection Error:**
- Updated `supabase-py` to version 2.0.3

**RLS Syntax Error:**
- Changed `current_user_id()` → `auth.uid()`

---

## Files Created

- `backend/config/supabase_client.py`
- `backend/.env.example`

---

## Environment Variables

```env
SUPABASE_URL=https://uretyyjnmyqqqxkxpwzl.supabase.co
SUPABASE_ANON_KEY=[anon-key]
SUPABASE_SERVICE_ROLE_KEY=[service-key]
```

---

## Testing

✅ Connection successful  
✅ Table created  
✅ RLS policies working  
✅ Email provider enabled

---

**Time:** 45 minutes
