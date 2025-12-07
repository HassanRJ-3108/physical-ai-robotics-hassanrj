# Test: Authentication Flow

**Feature**: 004-auth-system  
**Type**: End-to-End Test

## Test Setup

```bash
# Frontend
npm start

# Backend
cd backend
uv run uvicorn main:app --reload
```

## Test Cases

### TC-001: Sign Up Flow

**Steps:**
1. Navigate to http://localhost:3000/signup
2. Enter email: test@example.com
3. Enter password: SecurePass123
4. Click Sign Up button
5. Check email for verification link
6. Click verification link
7. Verify redirect to profile-setup

**Expected:**
- Sign up form submits successfully
- Email verification sent
- Verification link works
- Redirects to /profile-setup

**Status:** ✅ PASS

---

### TC-002: Profile Setup

**Steps:**
1. On profile-setup page
2. Select Programming Knowledge: "Intermediate"
3. Check Prior Robotics Experience: Yes
4. Select Learning Goals: ROS 2, Simulation
5. Select Learning Style: "Hands-on"
6. Click Complete Setup

**Expected:**
- Form submits to Supabase
- Data saved to user_profiles table
- Redirects to homepage
- User logged in

**Status:** ✅ PASS

---

### TC-003: Sign In Flow

**Steps:**
1. Navigate to http://localhost:3000/signin
2. Enter email: test@example.com
3. Enter password: SecurePass123
4. Click Sign In button

**Expected:**
- Authentication successful
- Redirects to homepage
- Navbar shows user avatar and name
- Chatbot shows personalized greeting

**Status:** ✅ PASS

---

### TC-004: Navbar Auth UI

**Steps:**
1. After login, check navbar
2. Click on user avatar/name
3. Verify dropdown menu appears
4. Check dropdown options

**Expected:**
- Avatar shows first letter of name
- Name displays correctly
- Dropdown has "Profile Setup" and "Logout"
- Dropdown closes on outside click

**Status:** ✅ PASS

---

### TC-005: Chatbot Personalization

**Steps:**
1. Open chatbot as logged-in user
2. Check welcome message
3. Verify profile info displayed

**Expected:**
- Header shows "Hi, {name}!"
- Welcome message shows programming level
- Welcome message shows learning goals
- No "Sign in" prompt for guests

**Status:** ✅ PASS

---

### TC-006: Auth Token in Requests

**Steps:**
1. Send chat message while logged in
2. Check Network tab in DevTools
3. Verify Authorization header present

**Expected:**
- Authorization: Bearer {token} header present
- Backend receives and validates token
- Response is personalized

**Status:** ✅ PASS

---

### TC-007: Logout

**Steps:**
1. Click user dropdown in navbar
2. Click "Logout"
3. Verify session cleared

**Expected:**
- User logged out
- Navbar shows "Sign In" / "Sign Up" buttons
- Chatbot shows guest prompt
- Redirects to homepage

**Status:** ✅ PASS

---

### TC-008: Protected Route

**Steps:**
1. Logout
2. Navigate directly to /profile-setup
3. Verify redirect behavior

**Expected:**
- Redirects to /signin if not authenticated
- After login, can access /profile-setup

**Status:** ✅ PASS

## Summary

**Total Tests:** 8  
**Passed:** 8  
**Failed:** 0  
**Coverage:** Signup, email verification, profile setup, login, navbar UI, chatbot personalization, logout, protected routes
