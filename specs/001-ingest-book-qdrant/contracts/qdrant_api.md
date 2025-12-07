# Qdrant API Interactions

This document outlines the expected interactions with the Qdrant API for the book ingestion process.

## Client Initialization
- **Action**: Initialize Qdrant client.
- **Input**: Qdrant host (URL), API key (optional).
- **Output**: Qdrant client instance.

## Collection Management

### Create Collection
- **Endpoint**: `qdrant_client.recreate_collection` (or `create_collection` if it doesn't exist)
- **Input**:
    - `collection_name`: `string` - The name of the collection.
    - `vector_size`: `integer` - Dimensionality of the vectors.
    - `distance_metric`: `string` - (e.g., `Cosine`, `Euclid`, `Dot`).
- **Output**: `boolean` - `True` on success, `False` on failure.
- **Error Taxonomy**: Collection already exists (handled by `recreate_collection`), invalid parameters, connection errors.

## Point Operations

### Upsert Points
- **Endpoint**: `qdrant_client.upsert`
- **Input**:
    - `collection_name`: `string` - The target collection.
    - `points`: `list` of `PointStruct` objects.
        - `PointStruct` attributes:
            - `id`: `integer` or `string` - Unique identifier for the point.
            - `vector`: `list of floats` - The embedding vector.
            - `payload`: `dictionary` - The associated metadata (e.g., `text`, `chunk_id`, `page_number`).
- **Output**: `UpdateResult` object indicating status (completed, failed) and operation ID.
- **Error Taxonomy**: Invalid points data, collection not found, connection errors, rate limits.

## Verification (Optional)

### Search Points (for post-ingestion verification)
- **Endpoint**: `qdrant_client.search`
- **Input**:
    - `collection_name`: `string` - The target collection.
    - `query_vector`: `list of floats` - The vector to search with.
    - `limit`: `integer` - Maximum number of results to return.
- **Output**: `list` of `ScoredPoint` objects.
    - `ScoredPoint` attributes:
        - `id`: `integer` or `string` - ID of the matched point.
        - `version`: `integer`
        - `score`: `float` - Similarity score.
        - `payload`: `dictionary` - The stored metadata.
        - `vector`: `list of floats` (optional, if requested).
- **Error Taxonomy**: Invalid query, collection not found, connection errors.
