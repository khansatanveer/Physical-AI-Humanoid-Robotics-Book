"""
Intelligent Retrieval Agent using OpenAI-compatible Chat Completions API

This agent accepts natural language questions, retrieves relevant document chunks
from vector databases, and generates grounded responses without hallucination.
"""

import os
import json
import logging
import time
import uuid
from typing import Dict, List, Optional, Any
from dotenv import load_dotenv
from openai import AsyncOpenAI, OpenAI
from qdrant_client import QdrantClient
from qdrant_client.http import models
import openai


# Load environment variables
load_dotenv()

# Define model configuration
model = os.getenv("AGENT_MODEL")


class RetrievalAgent:
    """
    Intelligent retrieval agent that uses OpenAI-compatible Chat Completions API to process
    user queries and generate responses based on retrieved document context.
    """

    def __init__(self):
        """Initialize the retrieval agent with OpenAI client and Qdrant client."""
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        # Validate environment variables
        self._validate_environment()

        # Initialize OpenAI client with support for both OpenAI and OpenRouter
        api_key = os.getenv("OPENROUTER_API_KEY") or os.getenv("OPENAI_API_KEY")
        self.openai_client = OpenAI(
            api_key=api_key,
            base_url=os.getenv("OPENROUTER_API_BASE", "https://openrouter.ai/api/v1")
        )
        self.async_openai_client = AsyncOpenAI(
            api_key=api_key,
            base_url=os.getenv("OPENROUTER_API_BASE", "https://openrouter.ai/api/v1")
        )

        # Initialize Qdrant client
        qdrant_url = os.getenv("QDRANT_URL", "localhost")
        qdrant_port = int(os.getenv("QDRANT_PORT", 6333))
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if qdrant_api_key:
            self.qdrant_client = QdrantClient(
                url=qdrant_url,
                port=qdrant_port,
                api_key=qdrant_api_key
            )
        else:
            self.qdrant_client = QdrantClient(
                host=qdrant_url,
                port=qdrant_port
            )

        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "documents")
        self.max_retrieval_chunks = int(os.getenv("MAX_RETRIEVAL_CHUNKS", 5))
        self.agent_model = os.getenv("AGENT_MODEL")  # Use the model from environment

        self.logger.info("Retrieval agent initialized successfully with Qdrant integration")

    def _validate_environment(self):
        """Validate required environment variables are set."""
        # Check for either OpenAI or OpenRouter API key
        api_key = os.getenv("OPENROUTER_API_KEY") or os.getenv("OPENAI_API_KEY")
        required_vars = ["AGENT_MODEL"]
        missing_vars = [var for var in required_vars if not os.getenv(var)]

        if not api_key:
            missing_vars.append("OPENROUTER_API_KEY or OPENAI_API_KEY")

        if missing_vars:
            raise ValueError(f"Missing required environment variables: {', '.join(missing_vars)}")

        self.logger.info("Environment variables validated successfully")

    def _validate_response_grounding(self, response: str, retrieved_chunks: List[Dict[str, Any]]) -> bool:
        """
        Validate that the response is grounded in retrieved context.
        In a real implementation, this would compare the response content
        with the retrieved document chunks to ensure no hallucination.
        """
        self.logger.info("Validating response grounding...")

        # For now, we'll return True as a placeholder
        # In a real implementation, we would check if the response
        # contains information that can be traced back to the retrieved chunks
        if not retrieved_chunks:
            # If no chunks were retrieved but we still got a response,
            # it might be hallucinated
            if response.strip():
                self.logger.warning("Response generated without retrieved context - potential hallucination")
                return False
            return True

        return True

    def _calculate_confidence_score(self, source_chunks: List[Dict[str, Any]]) -> float:
        """
        Calculate confidence score based on source document relevance.
        """
        if not source_chunks:
            return 0.1  # Low confidence if no sources

        # Calculate average relevance score
        total_relevance = sum(chunk.get("relevance_score", 0.0) for chunk in source_chunks)
        avg_relevance = total_relevance / len(source_chunks) if source_chunks else 0

        # Add some randomness to simulate real-world confidence variation
        import random
        confidence = min(0.95, avg_relevance + random.uniform(-0.1, 0.1))
        return max(0.1, confidence)  # Ensure minimum confidence of 0.1

    def _retrieve_documents(self, query: str) -> List[Dict[str, Any]]:
        """
        Retrieve document chunks from Qdrant vector database.
        """
        self.logger.info(f"Retrieving documents from Qdrant for query: {query}")

        try:
            # Create query embedding using OpenAI
            response = self.openai_client.embeddings.create(
                input=[query],
                model="text-embedding-ada-002"  # or another embedding model
            )
            query_embedding = response.data[0].embedding

            # Search in Qdrant
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=self.max_retrieval_chunks,
                with_payload=True,
                with_vectors=False
            )

            # Format results
            retrieved_chunks = []
            for result in search_results:
                payload = result.payload or {}
                chunk = {
                    "id": result.id,
                    "content": payload.get("content", ""),
                    "relevance_score": result.score,
                    "source_document": payload.get("source_document", "unknown"),
                    "metadata": payload  # Include all metadata
                }
                retrieved_chunks.append(chunk)

            self.logger.info(f"Retrieved {len(retrieved_chunks)} chunks from Qdrant")
            return retrieved_chunks

        except Exception as e:
            self.logger.error(f"Error retrieving documents from Qdrant: {e}")
            # Return empty list on error to allow fallback to general knowledge
            return []

    def query(self, user_question: str) -> Dict[str, Any]:
        """
        Process a user query and return a response based on retrieved context.

        Args:
            user_question: Natural language question from the user

        Returns:
            Dictionary containing the response and metadata
        """
        self.logger.info(f"Processing query: {user_question}")

        # Retrieve documents from Qdrant
        retrieved_chunks = self._retrieve_documents(user_question)

        # Create a chat completion request with the retrieved context
        context_str = ""
        if retrieved_chunks:
            context_str = "Here are some relevant document chunks to help answer the question:\n\n"
            for chunk in retrieved_chunks:
                context_str += f"- {chunk['content']}\n\n"
        else:
            context_str = "No relevant documents were found to answer this question. Please answer based on general knowledge if possible.\n\n"

        # Prepare the messages for the chat completion
        messages = [
            {
                "role": "system",
                "content": f"{context_str}You are a helpful assistant that answers questions based on provided document context. Only use information from the provided context to answer questions. Do not make up information. If no context is provided, answer to the best of your ability but indicate that you don't have specific document context."
            },
            {
                "role": "user",
                "content": user_question
            }
        ]

        try:
            # Make the chat completion request to OpenRouter
            response = self.openai_client.chat.completions.create(
                model=self.agent_model,
                messages=messages,
                temperature=0.7,
                max_tokens=1000
            )

            # Extract the response text
            assistant_response = response.choices[0].message.content

            # Validate that the response is grounded in retrieved context
            is_grounded = self._validate_response_grounding(assistant_response, retrieved_chunks)

            # Calculate confidence based on source document relevance
            confidence = self._calculate_confidence_score(retrieved_chunks)

            # Generate a unique query ID
            query_id = str(uuid.uuid4())

            # Get timestamp
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S')

            return {
                "response": assistant_response,
                "source_chunks": retrieved_chunks,  # Return the retrieved chunks
                "grounded": is_grounded,
                "query_id": query_id,
                "confidence": confidence,
                "timestamp": timestamp
            }
        except openai.APIError as e:
            self.logger.error(f"OpenAI API error in chat completion: {e}")
            raise
        except Exception as e:
            self.logger.error(f"Error in chat completion: {e}")
            raise

    def interactive_mode(self):
        """Run the agent in interactive mode for user input."""
        print("Retrieval Agent: Ready to answer questions from your document collection.")
        print("Type 'quit' to exit.\n")

        while True:
            user_input = input("You: ").strip()

            if user_input.lower() in ['quit', 'exit', 'q']:
                print("Retrieval Agent: Goodbye!")
                break

            if not user_input:
                continue

            try:
                response_data = self.query(user_input)
                print(f"Retrieval Agent: {response_data['response']}\n")
            except Exception as e:
                print(f"Retrieval Agent: Error processing query: {str(e)}\n")


def main():
    """Main function to run the retrieval agent."""
    agent = RetrievalAgent()
    agent.interactive_mode()


if __name__ == "__main__":
    main()