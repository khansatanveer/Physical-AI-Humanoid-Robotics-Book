// chatbot/hooks/useChat.js
import { useState, useCallback } from 'react';
import { queryGlobal, querySelection, validateQuery, validateSelectedText } from '../services/apiClient';

const useChat = () => {
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [currentMode, setCurrentMode] = useState('global'); // 'global' or 'selection'
  const [selectedText, setSelectedText] = useState('');

  // Add a message to the chat history
  const addMessage = useCallback((message) => {
    setMessages(prevMessages => [...prevMessages, message]);
  }, []);

  // Clear the chat history
  const clearChat = useCallback(() => {
    setMessages([]);
    setError(null);
  }, []);

  // Process a query based on the current mode
  const processQuery = useCallback(async (query) => {
    try {
      // Validate the query
      validateQuery(query);

      // Add user message to chat history
      const userMessage = {
        id: Date.now().toString(),
        role: 'user',
        content: query,
        timestamp: new Date().toISOString(),
        status: 'pending',
      };

      addMessage(userMessage);

      setIsLoading(true);
      setError(null);

      let response;
      if (currentMode === 'global') {
        response = await queryGlobal({
          query: query,
          session_id: 'session-' + Date.now(),
        });
      } else if (currentMode === 'selection' && selectedText) {
        validateSelectedText(selectedText);
        response = await querySelection({
          query: query,
          selected_text: selectedText,
          session_id: 'session-' + Date.now(),
        });
      } else if (currentMode === 'selection' && !selectedText) {
        throw new Error('No text selected for selection mode. Please select text on the page first.');
      } else {
        throw new Error('Invalid query mode');
      }

      // Add assistant response to chat history
      const assistantMessage = {
        id: response.query_id || Date.now().toString(),
        role: 'assistant',
        content: response.answer,
        sources: response.sources || [],
        timestamp: response.timestamp || new Date().toISOString(),
        status: 'success',
      };

      addMessage(assistantMessage);
    } catch (err) {
      console.error('Chat query error:', err);
      setError(err.message || 'An error occurred while processing your query');

      // Add error message to chat history
      const errorMessage = {
        id: Date.now().toString(),
        role: 'assistant',
        content: query || 'Previous query',
        timestamp: new Date().toISOString(),
        status: 'error',
      };

      addMessage(errorMessage);
    } finally {
      setIsLoading(false);
    }
  }, [currentMode, selectedText, addMessage]);

  // Set the current query mode
  const setMode = useCallback((mode) => {
    if (mode === 'global' || mode === 'selection') {
      setCurrentMode(mode);
    }
  }, []);

  // Update selected text
  const updateSelectedText = useCallback((text) => {
    setSelectedText(text);
  }, []);

  return {
    messages,
    isLoading,
    error,
    currentMode,
    selectedText,
    processQuery,
    addMessage,
    clearChat,
    setMode,
    updateSelectedText,
  };
};

export default useChat;