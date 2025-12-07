# Production Build & Deployment Guide

## ✅ SOLUTION: Backend on Railway, Frontend on Vercel

### Why This Works:

**Backend (Railway - Server Side):**
- ✅ Can use Supabase client (NO SSR issues - it's server-side!)
- ✅ FastAPI runs on server
- ✅ All auth logic here
- ✅ Database queries here

**Frontend (Vercel - Static/Client Side):**
- ✅ NO Supabase client import
- ✅ Only simple `fetch()` calls to backend
- ✅ NO SSR errors! 🎉
- ✅ Build will succeed!

---

## Backend Deployment (Railway)

### Step 1: Push to GitHub
```bash
git add .
git commit -m "Backend ready for deployment"
git push
```

### Step 2: Deploy on Railway
1. Go to https://railway.app
2. New Project → Deploy from GitHub
3. Select your repo
4. **Root Directory:** `/backend`
5. **Environment Variables:**
   ```
   GEMINI_API_KEY=your_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_key
   SUPABASE_URL=https://uretyyjnmyqqqxkxpwzl.supabase.co
   SUPABASE_SERVICE_ROLE_KEY=your_service_role_key
   ```
6. Railway auto-detects Python and runs `uvicorn main:app`
7. **Copy the deployment URL** (e.g., `https://your-app.railway.app`)

### Step 3: Test Backend
```bash
# Health check
curl https://your-app.railway.app/health

# Docs
https://your-app.railway.app/docs
```

---

## Frontend Deployment (Vercel)

### Step 1: Update Backend URL
Edit `src/lib/authAPI.ts` line 8:
```typescript
const API_URL = 'https://your-app.railway.app'; // Your Railway URL
```

### Step 2: Build Locally (Test)
```bash
npm run build
```
Should succeed now! ✅

### Step 3: Deploy on Vercel
1. Go to https://vercel.com
2. Import Git Repository
3. **Framework Preset:** Other
4. **Root Directory:** `/` (leave default)
5. **Build Command:** `npm run build`
6. **Output Directory:** `build`
7. Deploy!

---

## Environment Variables Summary

### Railway (Backend)
```env
GEMINI_API_KEY=xxx
QDRANT_URL=xxx
QDRANT_API_KEY=xxx
SUPABASE_URL=https://uretyyjnmyqqqxkxpwzl.supabase.co
SUPABASE_SERVICE_ROLE_KEY=xxx
```

### Vercel (Frontend)
No environment variables needed! Backend URL is hardcoded in `authAPI.ts`

(Optional: Can add `BACKEND_URL` if you want dynamic URL)

---

## API Endpoints (Backend)

Base URL: `https://your-app.railway.app`

### Auth
- `POST /api/auth/signup` - Register
- `POST /api/auth/signin` - Login
- `POST /api/auth/signout` - Logout
- `GET /api/auth/user?token=xxx` - Get user
- `PUT /api/auth/profile?token=xxx` - Update profile

### Chat
- `POST /api/chat` - Send message

### Health
- `GET /health` - Health check

---

## Why This Fixes SSR Issues

**Before (❌ Failed):**
```
Frontend → Supabase Client → localStorage/window
          ↑ SSR Error! window undefined during build
```

**After (✅ Works):**
```
Frontend → fetch() → Railway Backend → Supabase
           ↑ Just HTTP request, no SSR issues!
```

**Key Points:**
1. ✅ Supabase stays in backend (server-side, no issues)
2. ✅ Frontend uses simple fetch (works everywhere)
3. ✅ No browser APIs in frontend during build
4. ✅ Build succeeds, deploy succeeds!

---

## Testing Flow

### Development
```bash
# Terminal 1: Backend
cd backend
uv run uvicorn main:app --reload

# Terminal 2: Frontend
npm start
```

### Production
- Backend: https://your-app.railway.app
- Frontend: https://your-app.vercel.app
- Both talk to each other via HTTP!

---

## CORS Note

Backend already has CORS enabled for all origins. After deployment, update `main.py`:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://your-app.vercel.app",
        "http://localhost:3000"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

**DONE! Ready to deploy! 🚀**
