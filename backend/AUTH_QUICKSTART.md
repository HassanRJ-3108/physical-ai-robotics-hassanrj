# Authentication System - Quick Start Guide

Complete guide to use the new authentication system with Neon PostgreSQL.

## ✅ What's Been Set Up

Your authentication backend is ready with:
- **Neon PostgreSQL** database connected ✅
- **User accounts** with profiles
- **Session management** with JWT tokens
- **Auth endpoints** integrated into FastAPI
- **Password security** with Argon2 hashing

---

## 🚀 How to Use

### 1. Start the Server

```bash
cd backend
uv run python main.py
```

You should see:
```
📊 Connecting to database...
✅ Database connected successfully!
🤖 Initializing chatbot...
✅ Chatbot initialized successfully!
```

Server runs on: `http://localhost:8000`

---

## 📝 API Endpoints

### **POST** `/api/auth/signup` - Register New User

**Request:**
```json
{
  "email": "test@example.com",
  "password": "SecurePass123",
  "name": "Test User",
  "profile": {
    "programming_knowledge": "intermediate",
    "prior_robotics_experience": false,
    "learning_goals": ["ros2", "simulation", "ai"],
    "preferred_learning_style": "hands-on"
  }
}
```

**Response:**
```json
{
  "id": "uuid...",
  "email": "test@example.com",
  "name": "Test User",
  "programming_knowledge": "intermediate",
  "learning_goals": ["ros2", "simulation", "ai"],
  ...
}
```

**Try it with curl:**
```bash
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "SecurePass123",
    "name": "Test User",
    "profile": {
      "programming_knowledge": "intermediate",
      "prior_robotics_experience": false,
      "learning_goals": ["ros2", "simulation"],
      "preferred_learning_style": "hands-on"
    }
  }'
```

---

### **POST** `/api/auth/signin` - Login

**Request:**
```json
{
  "email": "test@example.com",
  "password": "SecurePass123"
}
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "bearer",
  "expires_in": 604800,
  "user": {
    "id": "uuid...",
    "email": "test@example.com",
    "name": "Test User"
  }
}
```

**Try it:**
```bash
curl -X POST http://localhost:8000/api/auth/signin \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "SecurePass123"}'
```

**Save the token** - You'll need it for authenticated requests!

---

### **POST** `/api/chat` - Chat (Works for Both Guest & Authenticated)

**Guest User (No Token):**
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is Physical AI?"}'
```

**Authenticated User (With Token):**
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_ACCESS_TOKEN_HERE" \
  -d '{"message": "What is Physical AI?"}'
```

Authenticated users get:
- ✅ Personalized responses
- ✅ Progress tracking
- ✅ Tailored difficulty level

---

## 🧪 Testing the Full Flow

### 1. Register a User
```bash
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "learner@example.com",
    "password": "MySecurePass123!",
    "name": "AI Learner",
    "profile": {
      "programming_knowledge": "beginner",
      "prior_robotics_experience": false,
      "learning_goals": ["ros2", "basics"],
      "preferred_learning_style": "theory-first"
    }
  }'
```

### 2. Login
```bash
curl -X POST http://localhost:8000/api/auth/signin \
  -H "Content-Type: application/json" \
  -d '{"email": "learner@example.com", "password": "MySecurePass123!"}'
```

Copy the `access_token` from response.

### 3. Chat with Authentication
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -d '{"message": "Explain ROS 2 to me"}'
```

---

## 🔐 Password Requirements

Passwords must:
- Be at least 8 characters
- Contain uppercase letter
- Contain lowercase letter
- Contain number

---

## 🗄️ Database Management

### View Users in Neon Dashboard
1. Go to https://console.neon.tech
2. Select your project
3. Click "SQL Editor"
4. Run: `SELECT * FROM users;`

### Check User Profiles
```sql
SELECT 
  u.email,
  u.name,
  p.programming_knowledge,
  p.learning_goals,
  p.total_questions_asked
FROM users u
JOIN user_profiles p ON u.id = p.user_id;
```

---

## 🐛 Troubleshooting

### "DATABASE_URL not found"
- Check `.env` file exists in `backend/` folder
- Verify DATABASE_URL is set correctly
- No quotes around the URL in .env

### "Failed to connect to database"
- Verify Neon project is active
- Check connection string is correct
- Ensure `sslmode=require` in URL

### "Invalid token"
- Token expired (7 days)
- Login again to get new token

### "Email already registered"
- User exists, try signin instead
- Or use different email

---

## 🎯 Next Steps

Now that auth is working:

1. **Frontend Integration** - Create signup/signin forms
2. **Personalized Chatbot** - Modify agent for user profiles
3. **Progress Tracking** - Show user's learning journey
4. **Profile Management** - Let users update their preferences

---

## 📚 API Documentation

Full interactive API docs available at:
- Swagger UI: `http://localhost:8000/docs`
- ReDoc: `http://localhost:8000/redoc`

---

## 🔑 Environment Variables

Your `.env` should have:
```bash
# Gemini API
GEMINI_API_KEY=AIzaSy...

# Neon Database
DATABASE_URL=postgresql://...

# Auth Security
AUTH_SECRET=random_32_character_string
```

**Security Note:** Never commit `.env` to git! It's in `.gitignore`.

---

## ✅ Status Check

Test everything works:
```bash
# 1. Database connection
uv run python -c "from database.connection import test_connection; import asyncio; asyncio.run(test_connection())"

# 2. Server starts
uv run python main.py

# 3. Health check
curl http://localhost:8000/health
```

All green? You're ready to go! 🎉
