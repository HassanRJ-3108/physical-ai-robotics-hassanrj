# Test: RAG Chatbot Functionality

**Feature**: 003-rag-chatbot  
**Type**: Integration Test

## Test Setup

```bash
cd backend
uv run uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

## Test Cases

### TC-001: Chat Endpoint Availability

**Steps:**
1. Send GET request to http://localhost:8000/healthz
2. Verify response

**Expected:**
- Status: 200 OK
- Response: `{"status": "healthy"}`

**Status:** ✅ PASS

---

### TC-002: Simple Chat Query

**Steps:**
1. Send POST to http://localhost:8000/api/chat
```json
{
  "message": "What is Physical AI?",
  "history": []
}
```
2. Verify response

**Expected:**
- Status: 200 OK
- Response contains relevant answer about Physical AI
- Response references book content

**Status:** ✅ PASS

---

### TC-003: RAG Context Retrieval

**Steps:**
1. Send query: "Explain ROS 2 communication"
2. Check response for technical accuracy
3. Verify context from book is used

**Expected:**
- Response mentions topics, publishers, subscribers
- Context retrieved from Qdrant
- Answer is technically accurate

**Status:** ✅ PASS

---

### TC-004: Conversation History

**Steps:**
1. Send first message: "What is simulation?"
2. Send follow-up: "Give me an example"
3. Verify context maintained

**Expected:**
- Second response refers to simulation
- History parameter works
- Conversation flows naturally

**Status:** ✅ PASS

---

### TC-005: Qdrant Connection

**Steps:**
1. Run data ingestion script
```bash
uv run python backend/ingest_data.py
```
2. Verify vectors uploaded
3. Test semantic search

**Expected:**
- Script completes without errors
- Vectors uploaded to Qdrant collection
- Semantic search returns relevant results

**Status:** ✅ PASS

---

### TC-006: Error Handling

**Steps:**
1. Send empty message
2. Send extremely long message (>10000 chars)
3. Send invalid JSON

**Expected:**
- Empty message: 422 Unprocessable Entity
- Long message: Handled gracefully or error
- Invalid JSON: 400 Bad Request

**Status:** ✅ PASS

---

### TC-007: CORS Configuration

**Steps:**
1. Send request from http://localhost:3000 origin
2. Verify CORS headers present

**Expected:**
- Access-Control-Allow-Origin header present
- Request succeeds from frontend origin

**Status:** ✅ PASS

## Summary

**Total Tests:** 7  
**Passed:** 7  
**Failed:** 0  
**Coverage:** API endpoints, RAG retrieval, conversation, error handling, CORS
