#!/usr/bin/env python3
"""
Test script to verify Qdrant connection and collection creation.
"""

import logging
from config import Config
from utils.storage import StorageManager

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def test_qdrant_connection():
    """Test Qdrant connection and collection creation."""
    logger.info("Starting Qdrant connection test...")

    # Validate configuration
    is_valid, msg = Config.validate()
    if not is_valid:
        logger.error(f"Configuration validation failed: {msg}")
        return False

    logger.info(f"Configuration is valid. Qdrant URL: {Config.QDRANT_URL}")
    logger.info(f"Collection name: {Config.QDRANT_COLLECTION_NAME}")

    try:
        # Initialize storage manager
        logger.info("Initializing StorageManager...")
        storage_manager = StorageManager()

        # Check existing collections
        logger.info("Getting list of existing collections...")
        collections = storage_manager.client.get_collections()
        logger.info(f"Existing collections: {[col.name for col in collections.collections]}")

        # Check if our collection exists
        collection_exists = any(col.name == storage_manager.collection_name for col in collections.collections)
        logger.info(f"Collection '{storage_manager.collection_name}' exists: {collection_exists}")

        # Set up the collection
        logger.info("Setting up collection...")
        storage_manager.setup_collection()

        # Check collections again
        collections = storage_manager.client.get_collections()
        logger.info(f"Updated collections list: {[col.name for col in collections.collections]}")

        # Verify our collection exists now
        collection_exists = any(col.name == storage_manager.collection_name for col in collections.collections)
        logger.info(f"Collection '{storage_manager.collection_name}' exists after setup: {collection_exists}")

        if collection_exists:
            logger.info("✅ SUCCESS: Qdrant collection is accessible!")

            # Get collection info
            collection_info = storage_manager.client.get_collection(storage_manager.collection_name)
            logger.info(f"Collection vectors count: {collection_info.points_count}")
            logger.info(f"Collection config: {collection_info.config}")

            return True
        else:
            logger.error("❌ FAILED: Collection was not created successfully")
            return False

    except Exception as e:
        logger.error(f"❌ ERROR: Failed to connect to Qdrant or create collection: {str(e)}")
        return False

if __name__ == "__main__":
    success = test_qdrant_connection()
    if success:
        logger.info("Test completed successfully!")
    else:
        logger.error("Test failed!")
        exit(1)