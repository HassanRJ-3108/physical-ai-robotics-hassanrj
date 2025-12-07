"""
Simple script to ingest book_knowledge.txt into Qdrant vector database.
Uses Gemini embeddings via OpenAI-compatible endpoint.
"""

import os
from pathlib import Path
from typing import List, Dict
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from openai import OpenAI
import re

load_dotenv()

# Qdrant Cloud Configuration
QDRANT_URL = "https://2ce7ad36-3328-4401-b371-1c7bd4262007.eu-west-2-0.aws.cloud.qdrant.io:6333"
QDRANT_API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.LKS6LL61MN0cr6D5Z_cjQBBh2FpHjP2cu4LKOwLvo24"
COLLECTION_NAME = "book_knowledge"

# Chunking settings
CHUNK_SIZE = 800  # words per chunk
CHUNK_OVERLAP = 100  # word overlap between chunks


def load_book(file_path: str) -> str:
    """Load book content"""
    with open(file_path, 'r', encoding='utf-8') as f:
        return f.read()


def extract_chapters(content: str) -> List[Dict[str, str]]:
    """Extract chapters from markdown"""
    chapter_pattern = r'^# (\d+\.\s+.+?)$'
    lines = content.split('\n')
    
    chapters = []
    current_chapter = None
    current_content = []
    
    for line in lines:
        match = re.match(chapter_pattern, line)
        if match:
            if current_chapter:
                chapters.append({
                    'title': current_chapter,
                    'content': '\n'.join(current_content).strip()
                })
            current_chapter = match.group(1)
            current_content = [line]
        else:
            if current_chapter:
                current_content.append(line)
    
    if current_chapter:
        chapters.append({
            'title': current_chapter,
            'content': '\n'.join(current_content).strip()
        })
    
    return chapters


def chunk_text(text: str, chapter: str) -> List[Dict]:
    """Split text into chunks with overlap"""
    chunks = []
    words = text.split()
    
    start = 0
    chunk_id = 0
    
    while start < len(words):
        end = start + CHUNK_SIZE
        chunk_words = words[start:end]
        chunk_text = ' '.join(chunk_words)
        
        chunks.append({
            'content': chunk_text,
            'chapter': chapter,
            'chunk_index': chunk_id
        })
        
        chunk_id += 1
        start = end - CHUNK_OVERLAP
        
        if start >= len(words):
            break
    
    return chunks


def generate_embedding(openai_client: OpenAI, text: str) -> List[float]:
    """Generate embedding using Gemini via OpenAI endpoint"""
    response = openai_client.embeddings.create(
        model="text-embedding-004",
        input=text
    )
    return response.data[0].embedding


def main():
    """Main ingestion pipeline"""
    print("🚀 Starting book ingestion to Qdrant Cloud...\n")
    
    # Get Gemini API key
    gemini_api_key = os.getenv('GEMINI_API_KEY')
    if not gemini_api_key:
        print("❌ GEMINI_API_KEY not found in .env file")
        return
    
    # Initialize OpenAI client with Gemini endpoint
    print("🔑 Initializing Gemini client...")
    openai_client = OpenAI(
        api_key=gemini_api_key,
        base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
    )
    
    # Initialize Qdrant client
    print("🔗 Connecting to Qdrant Cloud...")
    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY
    )
    
    # Load book
    book_path = Path(__file__).parent.parent / 'book_knowledge.txt'
    if not book_path.exists():
        print(f"❌ Book file not found: {book_path}")
        return
    
    print(f"📖 Loading book from: {book_path}")
    content = load_book(str(book_path))
    
    # Extract chapters
    print("📑 Extracting chapters...")
    chapters = extract_chapters(content)
    print(f"   Found {len(chapters)} chapters")
    
    # Chunk text
    print("✂️  Chunking text...")
    all_chunks = []
    for chapter in chapters:
        chunks = chunk_text(chapter['content'], chapter['title'])
        all_chunks.extend(chunks)
    print(f"   Generated {len(all_chunks)} chunks")
    
    # Get embedding dimension
    print("\n🔍 Getting embedding dimension...")
    sample_embedding = generate_embedding(openai_client, all_chunks[0]['content'])
    vector_size = len(sample_embedding)
    print(f"   Embedding dimension: {vector_size}")
    
    # Create collection
    print(f"\n🗄️  Setting up Qdrant collection '{COLLECTION_NAME}'...")
    try:
        collections = qdrant_client.get_collections().collections
        collection_names = [col.name for col in collections]
        
        if COLLECTION_NAME in collection_names:
            print(f"   ⚠️  Collection exists. Deleting...")
            qdrant_client.delete_collection(COLLECTION_NAME)
        
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE)
        )
        print(f"   ✅ Created collection")
    except Exception as e:
        print(f"   ❌ Error: {e}")
        return
    
    # Ingest chunks
    print(f"\n📥 Ingesting {len(all_chunks)} chunks...")
    points = []
    batch_size = 10
    
    for idx, chunk in enumerate(all_chunks):
        try:
            print(f"   Processing {idx + 1}/{len(all_chunks)}...", end='\r')
            
            embedding = generate_embedding(openai_client, chunk['content'])
            
            point = PointStruct(
                id=idx,
                vector=embedding,
                payload={
                    'content': chunk['content'],
                    'chapter': chunk['chapter'],
                    'chunk_index': chunk['chunk_index'],
                    'source': 'book_knowledge.txt'
                }
            )
            points.append(point)
            
            # Upload in batches
            if len(points) >= batch_size:
                qdrant_client.upsert(
                    collection_name=COLLECTION_NAME,
                    points=points
                )
                points = []
        
        except Exception as e:
            print(f"\n   ❌ Error on chunk {idx}: {e}")
            continue
    
    # Upload remaining points
    if points:
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )
    
    print(f"\n   ✅ Successfully ingested {len(all_chunks)} chunks!")
    
    # Verify
    collection_info = qdrant_client.get_collection(COLLECTION_NAME)
    print(f"\n📊 Collection Stats:")
    print(f"   Name: {COLLECTION_NAME}")
    print(f"   Points: {collection_info.points_count}")
    print(f"   Vector size: {collection_info.config.params.vectors.size}")
    print(f"   URL: {QDRANT_URL}")
    
    print("\n🎉 Ingestion complete!")


if __name__ == "__main__":
    main()
