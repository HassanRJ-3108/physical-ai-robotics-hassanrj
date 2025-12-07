# Book Knowledge RAG Chatbot Backend

Complete backend for a RAG (Retrieval-Augmented Generation) chatbot using:
- **OpenAI Agents SDK** with Gemini API
- **Qdrant Cloud** for vector storage
- **Gemini Embeddings** for semantic search

## 📁 Project Structure

```
backend/
├── chatbot/
│   ├── __init__.py
│   ├── agent.py              # RAG chatbot with OpenAI Agents SDK
│   └── qdrant_retriever.py   # Qdrant search functionality
├── ingest_data.py            # Script to ingest book into Qdrant
├── test_chatbot.py           # Test script for chatbot
├── pyproject.toml            # Dependencies
└── .env                      # Environment variables
```

## 🚀 Setup

### 1. Install Dependencies

```bash
uv sync
```

### 2. Configure Environment

Create `.env` file:

```bash
cp env.example .env
```

Add your Gemini API key:

```
GEMINI_API_KEY=your_actual_api_key_here
```

Get your API key from: https://aistudio.google.com/app/apikey

### 3. Ingest Book Data (One-time)

```bash
uv run python ingest_data.py
```

This will:
- Load `book_knowledge.txt`
- Extract chapters
- Create chunks with embeddings
- Upload to Qdrant Cloud

## 🤖 Using the Chatbot

### Test the Chatbot

```bash
uv run python test_chatbot.py
```

This will run sample queries and show responses.

### Use in Your Code

```python
import asyncio
from chatbot import BookChatbot

async def main():
    chatbot = BookChatbot()
    
    result = await chatbot.chat("What is Physical AI?")
    print(result['response'])

asyncio.run(main())
```

## 🏗️ Architecture

### 1. **Qdrant Retriever** (`chatbot/qdrant_retriever.py`)
- Connects to Qdrant Cloud
- Generates query embeddings using Gemini
- Performs semantic search
- Returns top-k relevant chunks

### 2. **RAG Agent** (`chatbot/agent.py`)
- Uses **OpenAI Agents SDK**
- Integrates Gemini via `OpenAIChatCompletionModel`
- Has `search_book` function tool
- Retrieves context and generates responses

### 3. **Flow**
```
User Query
    ↓
Agent receives query
    ↓
Agent calls search_book tool
    ↓
Qdrant Retriever:
  - Generates query embedding
  - Searches Qdrant
  - Returns relevant chunks
    ↓
Agent synthesizes response
    ↓
Returns answer to user
```

## 🔧 Key Features

✅ **OpenAI Agents SDK** - Modern agentic framework  
✅ **Gemini Integration** - Free Gemini API via OpenAIChatCompletionModel  
✅ **Qdrant Cloud** - Managed vector database  
✅ **RAG Pattern** - Retrieval-augmented generation  
✅ **Function Tools** - Agent can search book autonomously  
✅ **Async Support** - Fast, non-blocking operations

## 📊 Configuration

### Qdrant Settings (Hardcoded)
- **URL**: Qdrant Cloud instance
- **Collection**: `book_knowledge`
- **Vector Size**: 768 (Gemini embeddings)
- **Distance**: Cosine similarity

### Agent Settings
- **Model**: `gemini-2.0-flash-exp`
- **Temperature**: 0.7
- **Top-k Results**: 5 chunks per query

## 🧪 Testing

### Test Retriever Only

```bash
uv run python -c "from chatbot.qdrant_retriever import test_retriever; test_retriever()"
```

### Test Full Chatbot

```bash
uv run python test_chatbot.py
```

## 📝 Example Queries

- "What is Physical AI?"
- "Explain ROS 2 communication patterns"
- "How does bipedal locomotion work?"
- "Tell me about the NVIDIA Isaac Platform"
- "What is the Zero Moment Point?"

## 🔜 Next Steps

1. **FastAPI Backend** - Create REST API endpoints
2. **Frontend Integration** - Connect to Docusaurus
3. **Streaming Responses** - Add real-time streaming
4. **Conversation Memory** - Add chat history
5. **Production Deploy** - Deploy to cloud

## 📚 Documentation

- [OpenAI Agents SDK](https://openai.github.io/openai-agents-python/)
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [Gemini API](https://ai.google.dev/)
