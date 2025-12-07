# Plan: RAG Chatbot Backend

**Tech Stack:** FastAPI, Qdrant Cloud, Gemini API, OpenAI Agents SDK

## Implementation Steps

1. **FastAPI Setup**
   - Create backend/ directory
   - Initialize with `uv init`
   - Install dependencies (fastapi, uvicorn, qdrant-client, openai)

2. **Data Ingestion**
   - Create `book_knowledge.txt` with content
   - Implement text chunking (500 tokens, 50 overlap)
   - Generate embeddings using Gemini API
   - Upload to Qdrant collection

3. **RAG Pipeline**
   - Create retrieval function (semantic search)
   - Integrate OpenAI Agents SDK
   - Configure agent with Gemini model
   - Add retrieval tool to agent

4. **API Endpoints**
   - Create `POST /api/chat` endpoint
   - Define Pydantic models (ChatRequest, ChatResponse)
   - Add error handling
   - Configure CORS for frontend

5. **Testing**
   - Test data ingestion
   - Verify RAG retrieval accuracy
   - Test chat endpoint
   - Check response quality

## References

- [ADR 003](../../.specify/memory/adrs/003-fastapi-backend.md)
- [ADR 004](../../.specify/memory/adrs/004-qdrant-vector-db.md)
