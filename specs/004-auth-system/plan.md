# Plan: Authentication System

**Tech Stack:** Supabase Auth, Auth UI React, PostgreSQL

## Implementation Steps

1. **Supabase Setup**
   - Create Supabase project
   - Enable email provider
   - Create user_profiles table
   - Set up RLS policies

2. **Frontend - Auth Context**
   - Install @supabase/supabase-js, @supabase/auth-ui-react
   - Create src/lib/supabaseClient.ts
   - Create src/lib/AuthContext.tsx
   - Wrap app with AuthProvider

3. **Auth Pages**
   - Create src/pages/signin.tsx with Auth UI
   - Create src/pages/signup.tsx
   - Create src/pages/profile-setup.tsx
   - Style to match teal theme

4. **Navbar Integration**
   - Create src/components/NavbarAuth.tsx
   - Swizzle Navbar/Content.tsx
   - Add dropdown menu (Profile Setup, Logout)

5. **Chatbot Personalization**
   - Update ChatBot.tsx to use useAuth
   - Display user profile info
   - Send auth tokens with requests

6. **Backend Integration**
   - Install supabase-py
   - Create auth routes
   - Validate JWT tokens

## References

- [ADR 002](../../.specify/memory/adrs/002-supabase-auth.md)
