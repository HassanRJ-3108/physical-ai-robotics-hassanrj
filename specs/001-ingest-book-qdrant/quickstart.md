# Quickstart Guide: Ingest Book to Qdrant

This guide provides instructions to quickly set up and run the book ingestion script.

## Prerequisites

- Python 3.11 or later
- `pip` (or `pipenv`/`poetry` for virtual environment management)
- A running Qdrant instance (local or remote)
- The `book_knowledge.txt` file containing the content to be ingested.

## Setup

1.  **Clone the repository** (if not already done):
    ```bash
    git clone <repository_url>
    cd hackathon-project
    ```
2.  **Switch to the feature branch**:
    ```bash
    git checkout 001-ingest-book-qdrant
    ```
3.  **Create a virtual environment and install dependencies**:
    ```bash
    python -m venv .venv
    source .venv/Scripts/activate # On Windows
    # source .venv/bin/activate # On Linux/macOS
    pip install qdrant-client transformers tiktoken
    ```

## Usage

1.  **Place your book knowledge file**:
    Ensure your `book_knowledge.txt` file is in the root directory of the project.

2.  **Run the ingestion script**:
    Execute the script from the project root, providing your Qdrant host and the desired collection name.
    ```bash
    python src/cli/ingest_book.py --qdrant-host "http://localhost:6333" --collection-name "my_book_collection"
    ```
    Replace `"http://localhost:6333"` with your Qdrant instance URL and `"my_book_collection"` with your preferred collection name.

    *Optional arguments*:
    - `--api-key <YOUR_QDRANT_API_KEY>`: If your Qdrant instance requires an API key.
    - `--batch-size <SIZE>`: Number of points to upsert in a single batch (default might be 100).
    - `--vector-size <SIZE>`: Dimensionality of the vectors (default might be 384 for `all-MiniLM-L6-v2`).

## Verification

After running the script, you can verify the ingestion by querying your Qdrant instance. You can use the Qdrant client or the Qdrant UI to check the collection and search for points.

Example Python verification snippet:

```python
from qdrant_client import QdrantClient

client = QdrantClient(host="localhost", port=6333)
collection_name = "my_book_collection"

# Check collection info
collection_info = client.get_collection(collection_name=collection_name)
print(f"Collection {collection_name} info: {collection_info}")

# Perform a sample search (you'll need a query vector from your embedding model)
# from transformers import AutoModel, AutoTokenizer
# tokenizer = AutoTokenizer.from_pretrained("sentence-transformers/all-MiniLM-L6-v2")
# model = AutoModel.from_pretrained("sentence-transformers/all-MiniLM-L6-v2")
# # ... create a query_vector ...
# search_result = client.search(
#     collection_name=collection_name,
#     query_vector=query_vector,
#     limit=5,
#     with_payload=True
# )
# for hit in search_result:
#     print(f"ID: {hit.id}, Score: {hit.score}, Payload: {hit.payload}")
```
