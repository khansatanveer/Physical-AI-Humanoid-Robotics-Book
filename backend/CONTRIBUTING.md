# Contributing to RAG Chatbot Backend

Thank you for your interest in contributing to the RAG Chatbot Backend! This document outlines the process for contributing to this project.

## Project Structure

```
backend/
├── api/                 # API endpoints and routers
├── models/              # Data models
├── services/            # Business logic
├── utils/               # Utility functions
├── clients/             # External service clients
├── schemas/             # Request/response schemas
├── middleware/          # FastAPI middleware
├── tests/               # Test files
├── main.py              # Application entry point
├── config.py            # Configuration management
└── requirements.txt     # Python dependencies
```

## Getting Started

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/your-feature-name`
3. Set up the development environment:
   - Install Python 3.11
   - Install dependencies: `pip install -r requirements.txt`
   - Set up environment variables: `cp .env.example .env`
4. Make your changes
5. Run tests: `pytest`
6. Format code: `black . && isort .`
7. Commit your changes with descriptive commit messages
8. Push to your fork and submit a pull request

## Development Guidelines

### Code Style
- Follow PEP 8 guidelines
- Use type hints for all function parameters and return values
- Write docstrings for all public functions and classes
- Keep functions focused and single-purpose

### Testing
- Write unit tests for all business logic
- Write integration tests for API endpoints
- Maintain high test coverage (>80%)
- Use pytest for testing framework

### Architecture
- Follow the separation of concerns outlined in the README
- Keep business logic in service layer
- Use dependency injection where appropriate
- Follow FastAPI best practices

## Pull Request Process

1. Ensure your code follows the project's style and architecture guidelines
2. Add tests for new functionality
3. Update documentation as needed
4. Verify all tests pass
5. Submit a pull request with a clear description of changes
6. Link to any related issues

## Architecture Decisions

This project follows the principles outlined in the project constitution:
- Zero hallucination policy
- Context isolation between global and selected-text modes
- Deterministic and reproducible responses
- Transparency and source attribution

## Questions?

If you have questions about contributing, feel free to open an issue or contact the maintainers.