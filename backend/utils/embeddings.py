import cohere
from typing import List, Dict, Any
import logging
from config import Config

class EmbeddingsGenerator:
    """Utility for generating embeddings using Cohere API."""

    def __init__(self):
        if not Config.COHERE_API_KEY:
            raise ValueError("COHERE_API_KEY is required for embeddings generation")

        self.client = cohere.Client(Config.COHERE_API_KEY)
        self.model = "embed-multilingual-v3.0"  # Using multilingual model for book content
        logging.info("EmbeddingsGenerator initialized with Cohere client")

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for a list of text chunks."""
        logging.info(f"Starting embedding generation for {len(texts)} text chunks")
        try:
            # Cohere has limits on batch size, so we'll process in chunks
            all_embeddings = []
            batch_size = 96  # Cohere's limit is 96 texts per request

            num_batches = (len(texts) - 1) // batch_size + 1
            logging.info(f"Processing embeddings in {num_batches} batches of up to {batch_size} texts each")

            for i in range(0, len(texts), batch_size):
                batch = texts[i:i + batch_size]
                batch_num = i // batch_size + 1

                logging.debug(f"Processing embedding batch {batch_num}/{num_batches} with {len(batch)} texts")

                try:
                    response = self.client.embed(
                        texts=batch,
                        model=self.model,
                        input_type="search_document"  # Optimize for search documents
                    )

                    batch_embeddings = response.embeddings
                    all_embeddings.extend(batch_embeddings)

                    logging.info(f"Completed embedding generation for batch {batch_num}/{num_batches}")
                except cohere.CohereAPIError as e:
                    logging.error(f"Cohere API error in batch {batch_num}: {str(e)}")
                    raise
                except cohere.CohereConnectionError as e:
                    logging.error(f"Cohere connection error in batch {batch_num}: {str(e)}")
                    raise
                except Exception as e:
                    logging.error(f"Unexpected error generating embeddings in batch {batch_num}: {str(e)}")
                    raise

            logging.info(f"Successfully generated embeddings for {len(all_embeddings)} texts")
            return all_embeddings
        except Exception as e:
            logging.error(f"Error generating embeddings: {str(e)}")
            raise

    def generate_embedding_for_chunk(self, chunk_data: Dict) -> List[float]:
        """Generate embedding for a single content chunk."""
        content = chunk_data.get('content', '')
        if not content.strip():
            raise ValueError("Content cannot be empty for embedding generation")

        logging.debug(f"Generating embedding for chunk {chunk_data.get('id', 'unknown')} ({len(content)} chars)")

        try:
            response = self.client.embed(
                texts=[content],
                model=self.model,
                input_type="search_document"
            )

            logging.debug(f"Successfully generated embedding for chunk {chunk_data.get('id', 'unknown')}")
            return response.embeddings[0]  # Return the first (and only) embedding
        except cohere.CohereAPIError as e:
            logging.error(f"Cohere API error generating embedding for chunk {chunk_data.get('id', 'unknown')}: {str(e)}")
            raise
        except cohere.CohereConnectionError as e:
            logging.error(f"Cohere connection error generating embedding for chunk {chunk_data.get('id', 'unknown')}: {str(e)}")
            raise
        except Exception as e:
            logging.error(f"Error generating embedding for chunk {chunk_data.get('id', 'unknown')}: {str(e)}")
            raise

    def generate_embeddings_for_chunks(self, chunks: List[Dict]) -> List[Dict]:
        """Generate embeddings for a list of content chunks and return with metadata."""
        logging.info(f"Starting embedding generation for {len(chunks)} content chunks")
        if not chunks:
            logging.warning("No chunks provided for embedding generation")
            return []

        # Extract just the content texts for embedding generation
        texts = [chunk.get('content', '') for chunk in chunks]

        # Generate embeddings
        embeddings = self.generate_embeddings(texts)

        # Combine embeddings with chunk metadata
        result = []
        for i, chunk in enumerate(chunks):
            chunk_with_embedding = chunk.copy()
            chunk_with_embedding['embedding'] = embeddings[i]
            result.append(chunk_with_embedding)

        logging.info(f"Successfully added embeddings to {len(result)} chunks")
        return result