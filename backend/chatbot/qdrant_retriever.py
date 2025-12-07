"""
Qdrant retriever for RAG chatbot.
Searches the book_knowledge collection for relevant context.
"""

import os
from typing import List, Dict
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from openai import OpenAI

load_dotenv()

# Qdrant Cloud Configuration
QDRANT_URL = "https://2ce7ad36-3328-4401-b371-1c7bd4262007.eu-west-2-0.aws.cloud.qdrant.io:6333"
QDRANT_API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.LKS6LL61MN0cr6D5Z_cjQBBh2FpHjP2cu4LKOwLvo24"
COLLECTION_NAME = "book_knowledge"


class QdrantRetriever:
    """Retrieve relevant chunks from Qdrant for RAG"""
    
    def __init__(self):
        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY
        )
        
        # Initialize OpenAI client with Gemini endpoint for embeddings
        gemini_api_key = os.getenv('GEMINI_API_KEY')
        if not gemini_api_key:
            raise ValueError("GEMINI_API_KEY not found in .env")
        
        self.openai_client = OpenAI(
            api_key=gemini_api_key,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
        )
    
    def generate_query_embedding(self, query: str) -> List[float]:
        """Generate embedding for search query"""
        response = self.openai_client.embeddings.create(
            model="text-embedding-004",
            input=query
        )
        return response.data[0].embedding
    
    def search(self, query: str, top_k: int = 5) -> List[Dict]:
        """
        Search Qdrant for relevant chunks
        
        Args:
            query: User's question
            top_k: Number of results to return
        
        Returns:
            List of relevant chunks with metadata
        """
        try:
            # Generate query embedding
            query_vector = self.generate_query_embedding(query)
            
            # Search in Qdrant using query_points
            search_results = self.qdrant_client.query_points(
                collection_name=COLLECTION_NAME,
                query=query_vector,
                limit=top_k,
                with_payload=True
            )
            
            # Format results
            results = []
            for point in search_results.points:
                results.append({
                    'content': point.payload.get('content', ''),
                    'chapter': point.payload.get('chapter', ''),
                    'score': point.score,
                    'chunk_index': point.payload.get('chunk_index', 0)
                })
            
            return results
        
        except Exception as e:
            print(f"Error during search: {e}")
            return []
    
    def format_context(self, results: List[Dict]) -> str:
        """Format search results into context string"""
        if not results:
            return "No relevant information found."
        
        context_parts = []
        for i, result in enumerate(results, 1):
            chapter = result.get('chapter', 'Unknown')
            content = result.get('content', '')
            score = result.get('score', 0)
            
            context_parts.append(
                f"[Source {i} - {chapter}] (Relevance: {score:.2f})\n{content}"
            )
        
        return "\n\n---\n\n".join(context_parts)


# Test function
def test_retriever():
    """Test the retriever"""
    retriever = QdrantRetriever()
    
    test_queries = [
        "What is Physical AI?",
        "How does ROS 2 work?",
        "Tell me about humanoid robotics"
    ]
    
    for query in test_queries:
        print(f"\n{'='*60}")
        print(f"Query: {query}")
        print('='*60)
        
        results = retriever.search(query, top_k=3)
        context = retriever.format_context(results)
        print(context)


if __name__ == "__main__":
    test_retriever()
