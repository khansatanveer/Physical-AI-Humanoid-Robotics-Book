#!/usr/bin/env python3
"""
Book Website Embeddings Pipeline
Main entry point for the ingestion pipeline that crawls book websites,
extracts and chunks content, generates embeddings, and stores them in Qdrant.
"""

import logging
import sys
import argparse
from typing import List, Dict
from config import Config, validate_website_url
from utils.crawler import Crawler
from utils.chunker import Chunker
from utils.embeddings import EmbeddingsGenerator
from utils.storage import StorageManager

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def setup_pipeline():
    """Initialize all pipeline components."""
    logger.info("Initializing pipeline components...")

    # Validate configuration
    is_valid, msg = Config.validate()
    if not is_valid:
        logger.error(f"Configuration validation failed: {msg}")
        logger.error("Please check your .env file and ensure all required environment variables are set.")
        logger.error("Required variables: COHERE_API_KEY, QDRANT_URL, BOOK_WEBSITE_URL")
        sys.exit(1)

    # Initialize components
    try:
        logger.info("Initializing crawler...")
        crawler = Crawler()
        logger.info("Initializing chunker...")
        chunker = Chunker(Config.CHUNK_SIZE, Config.OVERLAP_SIZE)
        logger.info("Initializing embeddings generator...")
        embeddings_gen = EmbeddingsGenerator()
        logger.info("Initializing storage manager...")
        storage_manager = StorageManager()
    except ValueError as e:
        logger.error(f"Failed to initialize pipeline components: {str(e)}")
        logger.error("Please check your environment variables and ensure all required services are accessible.")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Unexpected error initializing pipeline components: {str(e)}")
        logger.error("Please verify your API keys and service availability.")
        sys.exit(1)

    # Set up Qdrant collection
    try:
        logger.info("Setting up Qdrant collection...")
        storage_manager.setup_collection()
        logger.info("Qdrant collection setup completed successfully.")
    except Exception as e:
        logger.error(f"Failed to set up Qdrant collection: {str(e)}")
        logger.error("Please verify your Qdrant URL and API key are correct and that Qdrant is accessible.")
        sys.exit(1)

    return crawler, chunker, embeddings_gen, storage_manager

def run_pipeline(target_url: str = None):
    """Run the complete ingestion pipeline."""
    import time
    start_time = time.time()

    # Use provided URL or fall back to config
    website_url = target_url or Config.BOOK_WEBSITE_URL

    if not website_url:
        logger.error("No website URL provided. Please set BOOK_WEBSITE_URL in config or provide via command line.")
        sys.exit(1)

    # Validate the website URL
    is_valid, msg = validate_website_url(website_url)
    if not is_valid:
        logger.error(f"Website validation failed: {msg}")
        sys.exit(1)

    logger.info(f"Starting pipeline for website: {website_url}")

    # Initialize pipeline components
    crawler, chunker, embeddings_gen, storage_manager = setup_pipeline()

    try:
        # Step 1: Crawl the website
        logger.info("Step 1: Crawling website...")
        crawl_start_time = time.time()
        content_pages = crawler.crawl_website(website_url)
        crawl_duration = time.time() - crawl_start_time
        logger.info(f"Crawling completed. Found {len(content_pages)} pages in {crawl_duration:.2f}s.")

        # Step 2: Process each page - chunk content
        logger.info("Step 2: Chunking content...")
        chunk_start_time = time.time()
        all_chunks = []
        for i, page in enumerate(content_pages):
            chunks = chunker.chunk_with_headings(page)
            all_chunks.extend(chunks)
            if (i + 1) % 10 == 0:  # Log progress every 10 pages
                logger.info(f"Processed {i + 1}/{len(content_pages)} pages for chunking...")
        chunk_duration = time.time() - chunk_start_time
        logger.info(f"Chunking completed. Generated {len(all_chunks)} content chunks in {chunk_duration:.2f}s.")

        # Step 3: Generate embeddings
        logger.info("Step 3: Generating embeddings...")
        embed_start_time = time.time()
        chunk_embeddings = embeddings_gen.generate_embeddings_for_chunks(all_chunks)
        embed_duration = time.time() - embed_start_time
        logger.info(f"Embeddings generated for {len(chunk_embeddings)} chunks in {embed_duration:.2f}s.")

        # Step 4: Store embeddings in Qdrant
        logger.info("Step 4: Storing embeddings in Qdrant...")
        storage_start_time = time.time()
        storage_manager.idempotent_store_embeddings(chunk_embeddings)
        storage_duration = time.time() - storage_start_time
        logger.info(f"Storage completed in {storage_duration:.2f}s.")

        total_duration = time.time() - start_time
        logger.info(f"Pipeline completed successfully in {total_duration:.2f}s!")

        # Create metrics summary
        metrics = {
            'pages_crawled': len(content_pages),
            'chunks_processed': len(all_chunks),
            'embeddings_stored': len(chunk_embeddings),
            'total_duration_seconds': total_duration,
            'crawl_duration_seconds': crawl_duration,
            'chunk_duration_seconds': chunk_duration,
            'embed_duration_seconds': embed_duration,
            'storage_duration_seconds': storage_duration,
            'chunks_per_second': len(all_chunks) / chunk_duration if chunk_duration > 0 else 0,
            'embeddings_per_second': len(chunk_embeddings) / embed_duration if embed_duration > 0 else 0
        }

        # Log summary metrics
        logger.info(f"Pipeline Metrics Summary:")
        logger.info(f"  - Pages Crawled: {metrics['pages_crawled']}")
        logger.info(f"  - Chunks Processed: {metrics['chunks_processed']}")
        logger.info(f"  - Embeddings Stored: {metrics['embeddings_stored']}")
        logger.info(f"  - Total Duration: {metrics['total_duration_seconds']:.2f}s")
        logger.info(f"  - Chunks/sec: {metrics['chunks_per_second']:.2f}")
        logger.info(f"  - Embeddings/sec: {metrics['embeddings_per_second']:.2f}")

        return metrics

    except Exception as e:
        logger.error(f"Pipeline failed with error: {str(e)}")
        raise

def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Book Website Embeddings Pipeline')
    parser.add_argument('--url', type=str, help='Target website URL to crawl (overrides config)')
    args = parser.parse_args()

    logger.info("Starting book embeddings pipeline...")

    results = run_pipeline(args.url)

    logger.info(f"Pipeline summary: {results['pages_crawled']} pages, "
                f"{results['chunks_processed']} chunks, "
                f"{results['embeddings_stored']} embeddings stored")

if __name__ == "__main__":
    main()