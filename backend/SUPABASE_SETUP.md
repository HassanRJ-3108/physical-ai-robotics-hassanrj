# Supabase Setup Guide - Quick Start

Get Supabase running in 5 minutes!

## Step 1: Create Supabase Project

1. **Go to**: https://supabase.com
2. **Click**: "Start your project"
3. **Sign in** with GitHub
4. **Create New Project**:
   - Name: `physical-ai-chatbot`
   - Database Password: (save this!)
   - Region: Choose closest to you
   - Click "Create new project"

⏳ Wait 2-3 minutes for setup...

## Step 2: Get Your Credentials

Once project is ready:

1. Click **"Project Settings"** (gear icon)
2. Click **"API"** in sidebar
3. Copy these 3 values:

```
Project URL: https://xxxxx.supabase.co
anon public key: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
service_role key: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

## Step 3: Run Database Schema

1. In Supabase dashboard, click **"SQL Editor"**
2. Click **"New query"**
3. Paste this SQL:

```sql
-- User profiles table
CREATE TABLE IF NOT EXISTS public.user_profiles (
  id UUID PRIMARY KEY REFERENCES auth.users(id) ON DELETE CASCADE,
  programming_knowledge TEXT CHECK (programming_knowledge IN ('beginner', 'intermediate', 'advanced')),
  prior_robotics_experience BOOLEAN DEFAULT false,
  learning_goals TEXT[] DEFAULT '{}',
  preferred_learning_style TEXT CHECK (preferred_learning_style IN ('hands-on', 'theory-first', 'mixed')),
  current_chapter TEXT,
  completed_chapters TEXT[] DEFAULT '{}',
  total_questions_asked INTEGER DEFAULT 0,
  interests JSONB DEFAULT '{}',
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Enable Row Level Security
ALTER TABLE public.user_profiles ENABLE ROW LEVEL SECURITY;

-- Users can view their own profile
CREATE POLICY "Users can view own profile"
  ON public.user_profiles
  FOR SELECT
  USING (auth.uid() = id);

-- Users can update their own profile
CREATE POLICY "Users can update own profile"
  ON public.user_profiles
  FOR UPDATE
  USING (auth.uid() = id);

-- Auto-create profile on signup
CREATE OR REPLACE FUNCTION public.handle_new_user()
RETURNS TRIGGER AS $$
BEGIN
  INSERT INTO public.user_profiles (id)
  VALUES (new.id);
  RETURN new;
END;
$$ LANGUAGE plpgsql SECURITY DEFINER;

CREATE TRIGGER on_auth_user_created
  AFTER INSERT ON auth.users
  FOR EACH ROW EXECUTE FUNCTION public.handle_new_user();
```

4. Click **"Run"**
5. You should see: "Success. No rows returned"

## Step 4: Update Environment Variables

Edit `backend/.env`:

```bash
# Supabase (NEW)
SUPABASE_URL=https://xxxxx.supabase.co
SUPABASE_SERVICE_ROLE_KEY=your_service_role_key_here

# Gemini (keep existing)
GEMINI_API_KEY=your_existing_gemini_key
```

For frontend, create `.env.local` in root:

```bash
NEXT_PUBLIC_SUPABASE_URL=https://xxxxx.supabase.co
NEXT_PUBLIC_SUPABASE_ANON_KEY=your_anon_public_key_here
```

## Step 5: Test Connection

```bash
cd backend
uv add supabase
uv run python -c "from supabase import create_client; import os; from dotenv import load_dotenv; load_dotenv(); client = create_client(os.getenv('SUPABASE_URL'), os.getenv('SUPABASE_SERVICE_ROLE_KEY')); print('✅ Connected to Supabase!')"
```

You should see: `✅ Connected to Supabase!`

## ✅ You're Done!

Supabase is now ready. Next steps:
1. I'll update the backend to use Supabase
2. Add frontend auth UI components
3. Test signup/signin

## Troubleshooting

**"Project creation failed"**
- Try different project name
- Check internet connection

**"SQL error"**
- Make sure you're in SQL Editor, not Table Editor
- Copy the entire SQL block

**"Connection test failed"**
- Double-check credentials in `.env`
- Make sure no extra spaces in URL/keys
- Verify `SUPABASE_SERVICE_ROLE_KEY` not `anon` key

---

**Ready?** Let me know when Supabase is set up and I'll migrate the code!
