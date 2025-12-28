// chatbot/services/apiClient.js
import axios from 'axios';

// Load API configuration
const API_BASE_URL =
    typeof window !== 'undefined'
        ? (process.env.REACT_APP_API_BASE_URL ||
           window.ENV?.REACT_APP_API_BASE_URL ||
           'http://localhost:8000')
        : process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';
const API_TIMEOUT =
    parseInt(process.env.REACT_APP_API_TIMEOUT, 10) || 30000;

// Axios instance
const apiClient = axios.create({
    baseURL: API_BASE_URL,
    timeout: API_TIMEOUT,
    headers: {
        'Content-Type': 'application/json',
    },
});

// ✅ Global query - uses the RAG backend /chat endpoint
export const queryGlobal = async({ query, session_id }) => {
    const res = await apiClient.post('/chat', {
        query,
        session_id,
    });

    // 🔑 NORMALIZED RESPONSE (useChat expects this format)
    return {
        response: res.data.response,
        sources: res.data.source_chunks || [], // Map source_chunks to sources
        confidence: res.data.confidence,
        status: 'success', // Add status since backend doesn't return it
        query_id: res.data.query_id,
        timestamp: res.data.timestamp,
    };
};

// ✅ Selection query - for context around selected text
export const querySelection = async({
    query,
    selected_text,
    session_id,
}) => {
    // For selection mode, we'll send the selected text as additional context
    // The backend can use this to focus on relevant sections
    const res = await apiClient.post('/chat', {
        query: `${query} (Context: ${selected_text.substring(0, 500)})`, // Add selected text as context
        session_id,
    });

    return {
        response: res.data.response,
        sources: res.data.source_chunks || [], // Map source_chunks to sources
        confidence: res.data.confidence,
        status: 'success', // Add status since backend doesn't return it
        query_id: res.data.query_id,
        timestamp: res.data.timestamp,
    };
};

// Validation
export const validateQuery = (query) => {
    if (!query || typeof query !== 'string')
        throw new Error('Query must be a string');
    if (query.length > 2000)
        throw new Error('Query cannot exceed 2000 characters');
};

export const validateSelectedText = (text) => {
    if (!text || typeof text !== 'string')
        throw new Error('Selected text must be a string');
    if (text.length > 5000)
        throw new Error('Selected text too long');
};

// Config export
export const getApiConfig = () => ({
    baseUrl: API_BASE_URL,
    timeout: API_TIMEOUT,
});