# RAG Chatbot Backend

## Spec

Build RAG-powered chatbot API with FastAPI for answering questions about book content.

**Goals:**
- Semantic search using Qdrant vector DB
- Gemini API for LLM responses
- Conversation history support
- REST API for frontend

## Plan

1. **Data Ingestion**: Chunk book text, generate embeddings, upload to Qdrant
2. **RAG Pipeline**: Semantic search + LLM generation
3. **API**: FastAPI with /api/chat endpoint
4. **Integration**: OpenAI Agents SDK + Gemini

**Tech:** FastAPI, Qdrant Cloud, Gemini API, OpenAI Agents SDK

## Tasks

- [x] Create FastAPI app
- [x] Implement text chunking
- [x] Generate embeddings (Gemini)
- [x] Upload to Qdrant
- [x] Build RAG retrieval
- [x] Integrate Agents SDK
- [x] Create /api/chat endpoint
- [x] Add CORS middleware
- [x] Test with sample queries

## Status

✅ Completed (2025-12-06)
