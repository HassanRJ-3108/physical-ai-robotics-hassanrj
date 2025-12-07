"""
Test script for the RAG chatbot.
Run this to test the chatbot with sample queries.
"""

import asyncio
from chatbot import BookChatbot


async def main():
    """Test the chatbot with sample queries"""
    print("🤖 Initializing Physical AI Book Chatbot...")
    print("=" * 70)
    
    try:
        chatbot = BookChatbot()
        print("✅ Chatbot initialized successfully!\n")
    except Exception as e:
        print(f"❌ Error initializing chatbot: {e}")
        return
    
    # Test queries
    queries = [
        "What is Physical AI and how is it different from traditional AI?",
        "Explain the main communication patterns in ROS 2",
        "What is the Zero Moment Point in bipedal locomotion?",
        "Tell me about the NVIDIA Isaac Platform",
    ]
    
    for i, query in enumerate(queries, 1):
        print(f"\n{'='*70}")
        print(f"Query {i}/{len(queries)}")
        print(f"{'='*70}")
        print(f"👤 User: {query}\n")
        
        result = await chatbot.chat(query)
        
        if result['success']:
            print(f"🤖 Assistant:\n{result['response']}")
        else:
            print(f"❌ Error: {result.get('error', 'Unknown error')}")
    
    print(f"\n{'='*70}")
    print("✅ Test complete!")
    print(f"{'='*70}")


if __name__ == "__main__":
    asyncio.run(main())
