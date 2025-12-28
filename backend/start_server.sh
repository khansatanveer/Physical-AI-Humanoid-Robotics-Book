#!/bin/bash
# Shell script to start the RAG Chat API server

echo "Starting RAG Chat API with OpenRouter integration..."
echo

# Check if Python is available
if ! command -v python &> /dev/null; then
    echo "Python is not installed or not in PATH"
    exit 1
fi

# Check if required packages are installed
python -c "import fastapi, openai, qdrant_client, python_dotenv" 2> /dev/null
if [ $? -ne 0 ]; then
    echo "Installing required packages..."
    pip install -r requirements.txt
fi

echo "Starting the API server on http://localhost:8000"
echo "Press Ctrl+C to stop the server"
echo
python api.py