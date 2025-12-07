# Prompt: Fix Backend CORS Error

**Date**: 2025-11-28  
**Agent**: Gemini CLI  
**Feature**: 003-rag-chatbot

---

## Problem

Frontend shows `ERR_CONNECTION_REFUSED` when trying to connect to chatbot backend.

---

## Debugging

1. Backend running on `localhost:8000` ✅
2. curl requests work ✅  
3. Browser requests fail ❌

**Network Tab showed:**
```
Status: (failed) net::ERR_CONNECTION_REFUSED
```

**Root Cause:** CORS misconfiguration

---

## Solution

### Fixed CORS Origins

Updated `backend/main.py`:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        "https://hassanrj-3108.github.io"
    ],
    allow_credentials=True,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["Content-Type", "Authorization"],
)
```

### Added Environment Variable Loading

```python
from dotenv import load_dotenv
load_dotenv()  # Must be at top of main.py
```

---

## Additional Fixes

**Port Already in Use:**
```bash
lsof -ti:8000 | xargs kill -9
```

**OPTIONS Handler:**
```python
@app.options("/api/chat")
async def chat_options():
    return {"ok": True}
```

---

## Testing

✅ CORS preflight works  
✅ Frontend connects successfully  
✅ Chat responses working  
✅ Environment variables loaded

---

## Files Modified

- `backend/main.py`
- `src/components/ChatBot.tsx`
- `backend/.env.example`

---

**Time:** 30 minutes
