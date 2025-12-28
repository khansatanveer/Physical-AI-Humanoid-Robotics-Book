# Data Model: Book Website Embeddings Pipeline

## Entities

### BookContent
- **id**: String (unique identifier based on URL + hash)
- **url**: String (source URL of the content)
- **title**: String (page title from HTML)
- **headings**: Array<String> (hierarchy of headings in the content)
- **content**: String (extracted text content)
- **metadata**: Object (additional metadata like section, tags, etc.)
- **created_at**: DateTime (timestamp of first processing)
- **updated_at**: DateTime (timestamp of last update)
- **content_hash**: String (hash of content for change detection)

### Embedding
- **id**: String (Qdrant vector ID, same as BookContent.id)
- **vector**: Array<Float> (Cohere embedding vector)
- **payload**: Object (contains BookContent fields for metadata)
- **collection_name**: String (Qdrant collection name)

### CrawlResult
- **id**: String (unique crawl session ID)
- **start_time**: DateTime (when crawl started)
- **end_time**: DateTime (when crawl ended)
- **urls_processed**: Array<String> (list of processed URLs)
- **urls_failed**: Array<String> (list of failed URLs)
- **total_pages**: Integer (number of pages processed)
- **status**: String (status of crawl: success, partial, failed)

### PipelineExecution
- **id**: String (unique execution ID)
- **start_time**: DateTime (execution start time)
- **end_time**: DateTime (execution end time)
- **status**: String (overall status: success, failed, partial)
- **metrics**: Object (processing metrics like pages_processed, embeddings_created)
- **errors**: Array<Object> (list of any errors encountered)

## Relationships

- One `PipelineExecution` contains multiple `CrawlResult` entries
- One `CrawlResult` may generate multiple `BookContent` entries
- One `BookContent` entry corresponds to one `Embedding` entry

## Validation Rules

- URL must be a valid HTTPS URL
- Content must not exceed Cohere embedding model limits (typically 4096 tokens)
- Content hash must be calculated using SHA256 algorithm
- Embedding vectors must match Cohere model dimensions (1024 for embed-multilingual-v3.0)
- Required fields (url, content, title) must not be empty

## State Transitions

- `PipelineExecution`: initiated → running → success/failed
- `BookContent`: extracted → chunked → embedded → stored