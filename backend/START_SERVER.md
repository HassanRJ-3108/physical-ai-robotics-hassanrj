# FastAPI Backend - Quick Start

## Start the Server

```bash
# From backend directory
uv run python main.py
```

Server will start on: `http://localhost:8000`

## Available Endpoints

- **GET** `/` - API information  
- **GET** `/health` - Health check  
- **POST** `/api/chat` - Send chat message  
- **POST** `/api/chat/reset` - Reset conversation  
- **GET** `/docs` - Interactive API documentation (Swagger UI)  
- **GET** `/redoc` - Alternative API documentation

## Test the API

### Using curl

```bash
# Health check
curl http://localhost:8000/health

# Send chat message
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is Physical AI?"}'
```

### Using Browser

Open: `http://localhost:8000/docs`

This will show the interactive Swagger UI where you can test all endpoints.

## Logs

The server logs all requests:
- `💬` Chat requests
- `✅` Successful responses  
- `❌` Errors

## Environment

Required `.env` file:
```bash
GEMINI_API_KEY=your_gemini_api_key_here
```

## CORS

Currently allows:
- `http://localhost:3000` (Docusaurus dev)
- `http://localhost:3001`
- All origins (`*`) for development

Restrict in production!
