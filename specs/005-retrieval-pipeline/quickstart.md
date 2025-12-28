# Quickstart: Retrieval Pipeline for Book Content

## Overview
This guide shows how to set up and use the retrieval pipeline for querying embedded book content using semantic similarity search.

## Prerequisites
- Python 3.11+
- Cohere API key
- Qdrant Cloud account and collection
- Git

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set up Environment
```bash
cd backend
cp .env.example .env
```

### 3. Update Environment Variables
Edit the `.env` file with your specific values:
```env
# Cohere API Configuration
COHERE_API_KEY="your-cohere-api-key-here"

# Qdrant Configuration
QDRANT_URL="your-qdrant-url-here"
QDRANT_API_KEY="your-qdrant-api-key-here"
QDRANT_COLLECTION_NAME=book_embeddings

# Book Website Configuration (for reference)
BOOK_WEBSITE_URL=https://your-book-website-url
```

### 4. Install Dependencies
```bash
pip install -r requirements.txt
```

## Usage

### Run the Retrieval Pipeline
```bash
python retrieve.py --query "your query text here" --top-k 5
```

### Command Line Options
- `--query`: The text query to search for (required)
- `--top-k`: Number of top results to return (default: 5)
- `--validate`: Run validation against known queries (optional)
- `--performance-test`: Run performance tests across various query types (optional)
- `--consistency-test`: Run consistency tests for repeated queries (optional)

### Example Usage
```bash
# Basic query with default top-5 results
python retrieve.py --query "What is physical AI?"

# Query with custom number of results
python retrieve.py --query "humanoid robot control systems" --top-k 10

# Query with validation
python retrieve.py --query "neural networks in robotics" --top-k 3 --validate

# Performance testing across multiple query types
python retrieve.py --query "test" --performance-test

# Consistency testing with repeated queries
python retrieve.py --query "What is physical AI?" --consistency-test
```

## Expected Output
The retrieval pipeline will return:
- Top-k most relevant content chunks with similarity scores
- Metadata for each chunk (URL, headings, content text)
- Performance metrics (query time, retrieval accuracy if validation enabled)

Example output:
```
Query: "What is physical AI?"
Query embedding generated in 0.5s
Retrieved 5 most similar chunks:
1. Score: 0.89 - "Physical AI combines..."
   URL: https://book-url/chapter1
   Headings: ["Introduction", "Definition"]
2. Score: 0.85 - "The field of physical AI..."
   URL: https://book-url/chapter2
   Headings: ["Foundations", "History"]
...
Total retrieval time: 1.2s
```

## Validation
To validate retrieval quality:
```bash
python retrieve.py --query "known query from book" --validate
```
This will compare results against expected answers and report accuracy metrics.

## Performance and Consistency Testing
To run performance tests:
```bash
python retrieve.py --query "test" --performance-test
```
This will run multiple queries and report performance metrics including response times and throughput.

To run consistency tests:
```bash
python retrieve.py --query "What is physical AI?" --consistency-test
```
This will run the same query multiple times and report consistency metrics.

## Troubleshooting
- **Connection errors**: Verify QDRANT_URL and QDRANT_API_KEY in .env
- **API errors**: Check COHERE_API_KEY in .env
- **No results**: Ensure the book_embeddings collection exists and has data
- **Slow performance**: Check network connectivity to Qdrant and Cohere APIs