# Supabase Email Confirmation Fix

## Problem
Sign-in is failing because Supabase requires email confirmation by default.

Your user shows: `Confirmed at: -` (NOT confirmed)

## Solution (Choose ONE)

### Option 1: Disable Email Confirmation (Recommended for Development)

1. Go to **Supabase Dashboard**: https://supabase.com/dashboard
2. Select your project
3. Click **Authentication** (left sidebar)
4. Click **Providers**
5. Click **Email**
6. **Turn OFF** "Confirm email"
7. Click **Save**

Then delete the old user and signup again.

### Option 2: Manually Confirm Existing User

1. Go to **Supabase Dashboard**
2. Click **Authentication** → **Users**
3. Find `huzaifa3108hassan@gmail.com`
4. Click the **three dots menu** (⋮)
5. Click **"Send magic link"** or manually set confirmed

### Option 3: Check Your Email

Look for confirmation email from Supabase in:
- Inbox
- Spam/Junk folder

Click the confirmation link.

---

## After Fixing

Refresh the page and try signin again with:
- Email: `huzaifa3108hassan@gmail.com`
- Password: `String123`
