@echo off
REM Windows batch script to start the RAG Chat API server

echo Starting RAG Chat API with OpenRouter integration...
echo.

REM Check if Python is available
python --version >nul 2>&1
if errorlevel 1 (
    echo Python is not installed or not in PATH
    pause
    exit /b 1
)

REM Check if required packages are installed
python -c "import fastapi, openai, qdrant_client, python_dotenv" >nul 2>&1
if errorlevel 1 (
    echo Installing required packages...
    pip install -r requirements.txt
)

echo Starting the API server on http://localhost:8000
echo Press Ctrl+C to stop the server
echo.
python api.py