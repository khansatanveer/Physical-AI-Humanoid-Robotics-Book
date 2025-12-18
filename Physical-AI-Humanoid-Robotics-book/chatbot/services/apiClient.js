// chatbot/services/apiClient.js
import axios from 'axios';

// Load API configuration from environment variables
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';
const API_TIMEOUT = parseInt(process.env.REACT_APP_API_TIMEOUT) || 30000;

// Create axios instance with default configuration
const apiClient = axios.create({
  baseURL: API_BASE_URL,
  timeout: API_TIMEOUT,
  headers: {
    'Content-Type': 'application/json',
  },
});

// Global query function
export const queryGlobal = async (request) => {
  try {
    const response = await apiClient.post('/query', {
      query: request.query,
      user_id: request.user_id,
      session_id: request.session_id,
    });

    return response.data;
  } catch (error) {
    console.error('Error in global query:', error);
    throw error;
  }
};

// Selection query function
export const querySelection = async (request) => {
  try {
    const response = await apiClient.post('/query/selection', {
      query: request.query,
      selected_text: request.selected_text,
      user_id: request.user_id,
      session_id: request.session_id,
    });

    return response.data;
  } catch (error) {
    console.error('Error in selection query:', error);
    throw error;
  }
};

// Validation functions
export const validateQuery = (query) => {
  if (!query || typeof query !== 'string') {
    throw new Error('Query is required and must be a string');
  }

  if (query.trim().length < 1) {
    throw new Error('Query must not be empty');
  }

  if (query.length > 2000) {
    throw new Error('Query must be 2000 characters or less');
  }

  return true;
};

export const validateSelectedText = (selectedText) => {
  if (!selectedText || typeof selectedText !== 'string') {
    throw new Error('Selected text is required and must be a string');
  }

  if (selectedText.length > 5000) {
    throw new Error('Selected text must be 5000 characters or less');
  }

  return true;
};

// Export API configuration for use in other modules
export const getApiConfig = () => ({
  baseUrl: API_BASE_URL,
  timeout: API_TIMEOUT,
});