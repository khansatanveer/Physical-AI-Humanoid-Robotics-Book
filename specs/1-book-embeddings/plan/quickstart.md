# Quickstart: Book Website Embeddings Pipeline

## Prerequisites

- Python 3.11+
- UV package manager
- Access to Cohere API
- Access to Qdrant Cloud (free tier)

## Setup

1. **Create backend directory and setup environment:**
```bash
mkdir backend
cd backend
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
```

2. **Create requirements.txt:**
```txt
requests==2.31.0
beautifulsoup4==4.12.2
cohere==5.5.8
qdrant-client==1.9.2
python-dotenv==1.0.0
tqdm==4.66.1
lxml==4.9.3
```

3. **Install dependencies:**
```bash
uv pip install -r requirements.txt
```

4. **Create environment file:**
```bash
cp .env.example .env
```

5. **Configure environment variables:**
```env
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=book_embeddings
BOOK_WEBSITE_URL=https://your-book-website.com
```

## Run the Pipeline

1. **Execute the main pipeline:**
```bash
cd backend
python main.py
```

2. **Or run with specific URL:**
```bash
python main.py --url https://your-book-website.com
```

## Expected Output

- Content crawled from the book website
- Content chunked with metadata preserved
- Embeddings generated and stored in Qdrant
- Logging showing progress of each stage
- Completion summary with metrics

## Verification

After running the pipeline:
1. Check Qdrant dashboard to confirm embeddings are stored
2. Verify collection contains expected number of vectors
3. Check that metadata includes URL, headings, and content properly