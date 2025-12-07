# Tasks: Authentication System

## Supabase Setup
- [x] Create Supabase project
- [x] Enable email provider
- [x] Create user_profiles table
- [x] Set up RLS policies

## Frontend - Dependencies
- [x] Install @supabase/supabase-js
- [x] Install @supabase/auth-ui-react
- [x] Install @supabase/auth-ui-shared

## Frontend - Auth Components
- [x] Create supabaseClient.ts
- [x] Create AuthContext.tsx
- [x] Wrap app with AuthProvider
- [x] Update Root.tsx

## Auth Pages
- [x] Create signin.tsx + styles
- [x] Create signup.tsx + styles
- [x] Create profile-setup.tsx + styles
- [x] Test auth flow

## Navbar
- [x] Create NavbarAuth.tsx
- [x] Create NavbarAuth.module.css
- [x] Swizzle Navbar/Content.tsx
- [x] Add dropdown menu
- [x] Test mobile view

## Chatbot
- [x] Update ChatBot.tsx with useAuth
- [x] Display user profile info
- [x] Send auth tokens
- [x] Remove sign-out button

## Backend
- [x] Install supabase-py
- [x] Create auth/routes.py
- [x] Add signup/signin endpoints
- [x] Test JWT validation

## Testing
- [x] Test signup flow
- [x] Test email verification
- [x] Test profile setup
- [x] Test signin
- [x] Test logout
- [x] Verify chatbot personalization
