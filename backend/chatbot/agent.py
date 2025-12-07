"""
RAG Chatbot using OpenAI Agents SDK with Gemini and Qdrant.
Uses OpenAIChatCompletionModel to integrate Gemini API.
"""

import os
import asyncio
from typing import List, Dict
from dotenv import load_dotenv
from agents import AsyncOpenAI
from agents import Agent, Runner, function_tool
from agents import OpenAIChatCompletionsModel
from .qdrant_retriever import QdrantRetriever

load_dotenv()


class BookChatbot:
    """RAG Chatbot for Physical AI & Robotics Book"""
    
    def __init__(self):
        # Get Gemini API key
        gemini_api_key = 'AIzaSyBiiSje7yzzZZ8CNCbpcGXKcBWTDv8Azx0'
        if not gemini_api_key:
            raise ValueError("GEMINI_API_KEY not found in .env")
        
        # Initialize Qdrant retriever
        self.retriever = QdrantRetriever()
        
        # Create AsyncOpenAI client with Gemini endpoint
        gemini_client = AsyncOpenAI(
            api_key=gemini_api_key,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
        )
        
        # Create OpenAIChatCompletionModel with Gemini
        self.model = OpenAIChatCompletionsModel(
            model="gemini-2.0-flash",
            openai_client=gemini_client,
        )
        
        # Define search tool for the agent
        @function_tool
        def search_book(query: str) -> str:
            """
            Search the Physical AI & Robotics book for relevant information.
            
            Args:
                query: The search query to find relevant book content
            
            Returns:
                Relevant excerpts from the book with chapter information
            """
            results = self.retriever.search(query, top_k=5)
            return self.retriever.format_context(results)
        
        # Store the tool
        self.search_tool = search_book
        
        # Create the agent
        self.agent = Agent(
            name="Physical AI Book Assistant",
            model=self.model,
            instructions=self._get_instructions(),
            tools=[self.search_tool]
        )
    
    def _get_instructions(self) -> str:
        """Get system instructions for the agent"""
        return """You are an expert AI assistant specializing in Physical AI and Humanoid Robotics.

You have access to a comprehensive book covering:
- Physical AI Fundamentals and Embodied Intelligence
- ROS 2 (Robot Operating System)
- Robot Simulation (Gazebo, Unity, Isaac Sim)
- NVIDIA Isaac Platform
- Vision-Language-Action (VLA) Models
- Humanoid Robotics and Bipedal Locomotion
- Hardware Setup and Edge Computing

Your role:
1. Answer questions using the search_book tool to find relevant information
2. Provide accurate, detailed explanations based on the book content
3. Cite specific chapters when providing information
4. If a question is outside the book's scope, politely inform the user
5. Be helpful, clear, and educational

Guidelines:
- ALWAYS use the search_book tool first to find relevant information
- Synthesize information from multiple sources when needed
- Provide practical examples when appropriate
- Reference specific chapters and sections
- If unsure, say so rather than making up information
- Keep responses concise but comprehensive

Remember: You are a teaching assistant helping users learn about Physical AI and Robotics."""
    
    async def chat(self, message: str) -> Dict:
        """
        Process a chat message
        
        Args:
            message: User's message
        
        Returns:
            Dictionary with response and metadata
        """
        try:
            # Run the agent
            result = await Runner.run(
                self.agent,
                message
            )
            
            return {
                "response": result.final_output,
                "success": True
            }
        
        except Exception as e:
            return {
                "response": f"I apologize, but I encountered an error: {str(e)}",
                "success": False,
                "error": str(e)
            }
    
    async def chat_stream(self, message: str):
        """
        Stream chat responses (for future use)
        
        Args:
            message: User's message
        
        Yields:
            Response chunks
        """
        try:
            result = await Runner.run(
                self.agent,
                message,
                stream=True
            )
            
            async for chunk in result:
                if hasattr(chunk, 'content') and chunk.content:
                    yield chunk.content
        
        except Exception as e:
            yield f"Error: {str(e)}"


# Test function
async def test_chatbot():
    """Test the chatbot"""
    print("🤖 Initializing Book Chatbot...\n")
    
    chatbot = BookChatbot()
    
    test_queries = [
        "What is Physical AI?",
        "Explain how ROS 2 communication works",
        "What are the challenges in bipedal locomotion?"
    ]
    
    for query in test_queries:
        print(f"\n{'='*60}")
        print(f"👤 User: {query}")
        print('='*60)
        
        result = await chatbot.chat(query)
        
        if result['success']:
            print(f"\n🤖 Assistant:\n{result['response']}")
        else:
            print(f"\n❌ Error: {result.get('error', 'Unknown error')}")
        
        print("\n" + "="*60)


if __name__ == "__main__":
    asyncio.run(test_chatbot())
