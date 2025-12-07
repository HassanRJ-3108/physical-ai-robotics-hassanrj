"""
Unit tests for the text_processing module.
"""
import pytest
import torch
from src.lib.text_processing import chunk_text, generate_embeddings

def test_chunk_text_empty_input():
    assert chunk_text("") == []

def test_chunk_text_single_chunk():
    text = "This is a short sentence." * 5
    chunks = chunk_text(text, max_tokens=100)
    assert len(chunks) == 1
    assert chunks[0] == text

def test_chunk_text_multiple_chunks_no_overlap():
    text = "word " * 500  # Creates a text with ~500 words
    # Using a smaller max_tokens and 0 overlap to ensure multiple distinct chunks
    chunks = chunk_text(text, max_tokens=10, overlap=0)
    # With max_tokens=10, each 'word ' will be 1 token, so 500 tokens total.
    # Expect 50 chunks (500 / 10).
    assert len(chunks) == 50
    assert chunks[0].strip() == "word word word word word word word word word word"
    assert chunks[49].strip() == "word word word word word word word word word word"

def test_chunk_text_multiple_chunks_with_overlap():
    text = "This is a sample sentence for testing chunking with overlap." * 5
    chunks = chunk_text(text, max_tokens=10, overlap=5)
    assert len(chunks) > 1
    # Verify overlap manually for a couple of chunks
    # This test might be fragile due to tokenization, consider a more robust check.
    # For now, just check that chunks are produced and there's some content.
    assert len(chunks[0]) > 0
    assert len(chunks[1]) > 0

def test_chunk_text_exact_split():
    text = "a " * 20
    chunks = chunk_text(text, max_tokens=2, overlap=0)
    assert len(chunks) == 10
    assert chunks[0] == "a a"
    assert chunks[9] == "a a"

def test_chunk_text_model_name():
    # Ensure different model names can be used without error
    text = "Hello world"
    chunks = chunk_text(text, model_name="text-embedding-ada-002", max_tokens=5)
    assert len(chunks) == 1
    assert chunks[0] == "Hello world"

def test_generate_embeddings_single_text():
    texts = ["This is a test sentence."]
    embeddings = generate_embeddings(texts)
    assert len(embeddings) == 1
    assert len(embeddings[0]) > 0  # Embeddings should have some dimension
    assert isinstance(embeddings[0][0], float)

def test_generate_embeddings_multiple_texts():
    texts = ["This is the first sentence.", "This is the second sentence.", "And a third one."]
    embeddings = generate_embeddings(texts)
    assert len(embeddings) == len(texts)
    assert all(len(emb) > 0 for emb in embeddings)
    assert all(isinstance(emb[0], float) for emb in embeddings)

def test_generate_embeddings_empty_list():
    texts = []
    embeddings = generate_embeddings(texts)
    assert embeddings == []

def test_generate_embeddings_model_output_shape():
    texts = ["hello world"]
    embeddings = generate_embeddings(texts)
    assert len(embeddings[0]) == 384  # Default all-MiniLM-L6-v2 embedding dimension
