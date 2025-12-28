# Quickstart: Intelligent Retrieval Agent

## Prerequisites

- Python 3.11 or higher
- OpenAI API key
- Access to the existing retrieval pipeline/vector database
- pip package manager

## Setup

1. **Install Dependencies**
   ```bash
   pip install openai python-dotenv
   ```

2. **Configure Environment**
   Create a `.env` file in the project root:
   ```env
   OPENAI_API_KEY=your_openai_api_key_here
   # Add any other required configuration for the retrieval pipeline
   ```

3. **Prepare Document Collection**
   Ensure your documents are indexed in the vector database through the existing retrieval pipeline.

## Basic Usage

1. **Run the Agent**
   ```bash
   python agent.py
   ```

2. **Interactive Mode**
   The agent will start in interactive mode where you can ask questions:
   ```
   Retrieval Agent: Ready to answer questions from your document collection.
   You: What are the key principles of humanoid robotics?
   Retrieval Agent: Based on the retrieved documents, the key principles of humanoid robotics include...
   ```

3. **API Mode**
   The agent also exposes a REST API endpoint at `/query`:
   ```bash
   curl -X POST http://localhost:8000/query \
     -H "Content-Type: application/json" \
     -d '{"query": "What are the key principles of humanoid robotics?"}'
   ```

## Configuration

The agent can be configured through environment variables:

- `OPENAI_API_KEY`: Your OpenAI API key (required)
- `VECTOR_DB_ENDPOINT`: Endpoint for the vector database (if applicable)
- `MAX_RETRIEVAL_CHUNKS`: Maximum number of document chunks to retrieve (default: 5)
- `AGENT_MODEL`: OpenAI model to use (default: gpt-4-turbo)

## Example Queries

1. **Factual Question**
   ```
   Query: What is the definition of embodied cognition?
   Response: According to the retrieved documents, embodied cognition is defined as...
   Sources: [document1.pdf, document2.pdf]
   ```

2. **Question with No Relevant Documents**
   ```
   Query: What is the weather like today?
   Response: I cannot answer this question as it is not covered in the document collection.
   Sources: []
   ```

## Troubleshooting

- **API Key Issues**: Verify that `OPENAI_API_KEY` is set correctly in your environment
- **Retrieval Failures**: Check that the document collection is properly indexed
- **Response Quality**: If responses seem inaccurate, verify that the retrieved chunks contain relevant information