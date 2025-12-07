# Data Model: Ingest Book to Qdrant

This document defines the key data entities involved in the process of ingesting book content into Qdrant.

## Entities

### Book Content
- **Description**: Represents the raw textual content of the book from the input file.
- **Attributes**:
    - `raw_text`: `string` - The complete text content of the book.
    - `source_file`: `string` - The name of the original file (e.g., `book_knowledge.txt`).

### Text Chunk
- **Description**: A segment of the `Book Content` suitable for individual embedding. Chunks are typically created to fit within an embedding model's token limit.
- **Attributes**:
    - `text`: `string` - The actual text content of the chunk.
    - `chunk_id`: `string` (UUID/hash) - A unique identifier for the chunk.
    - `start_offset`: `integer` - The starting character index of the chunk in the original `raw_text`.
    - `end_offset`: `integer` - The ending character index of the chunk in the original `raw_text`.
    - `metadata`: `dictionary` - Additional information about the chunk (e.g., `{'page_number': 1, 'chapter_title': 'Introduction'}`).

### Embedding
- **Description**: The numerical vector representation of a `Text Chunk`.
- **Attributes**:
    - `vector`: `list of floats` - The high-dimensional vector representing the semantic meaning of the `Text Chunk`.
    - `model_used`: `string` - The identifier of the embedding model used to generate this vector (e.g., `all-MiniLM-L6-v2`).

### Qdrant Collection
- **Description**: A logical container within the Qdrant vector database where the `Embeddings` and their associated `Text Chunk` metadata are stored.
- **Attributes**:
    - `name`: `string` - The unique name of the collection.
    - `vector_size`: `integer` - The dimensionality of the vectors stored in this collection (must match the embedding model output).
    - `distance_metric`: `string` - The metric used to calculate the similarity between vectors (e.g., `Cosine`, `Euclid`, `Dot`).
    - `on_disk_payload`: `boolean` - Whether to store payload on disk (default `true`).
