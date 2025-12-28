// chatbot/hooks/useChat.js
import { useState, useCallback } from 'react';
import {
    queryGlobal,
    querySelection,
    validateQuery,
    validateSelectedText,
} from '../services/apiClient';

const useChat = () => {
    const [messages, setMessages] = useState([]);
    const [isLoading, setIsLoading] = useState(false);
    const [error, setError] = useState(null);
    const [currentMode, setCurrentMode] = useState('global');
    const [selectedText, setSelectedText] = useState('');

    const addMessage = useCallback((message) => {
        setMessages((prev) => [...prev, message]);
    }, []);

    const processQuery = useCallback(
        async(query) => {
            try {
                validateQuery(query);

                // USER MESSAGE
                addMessage({
                    id: Date.now().toString(),
                    role: 'user',
                    content: query,
                    timestamp: new Date().toISOString(),
                });

                setIsLoading(true);
                setError(null);

                let response;

                if (currentMode === 'global') {
                    response = await queryGlobal({
                        query,
                        session_id: `session-${Date.now()}`,
                    });
                } else {
                    validateSelectedText(selectedText);
                    response = await querySelection({
                        query,
                        selected_text: selectedText,
                        session_id: `session-${Date.now()}`,
                    });
                }

                // ✅ ASSISTANT MESSAGE (FIXED)
                addMessage({
                    id: Date.now().toString(),
                    role: 'assistant',
                    content: response.response, // Now correctly maps to response from API client
                    sources: response.sources || [],
                    confidence: response.confidence,
                    timestamp: new Date().toISOString(),
                });
            } catch (err) {
                console.error(err);
                setError(err.message || 'Chat error');

                addMessage({
                    id: Date.now().toString(),
                    role: 'assistant',
                    content: '❌ Something went wrong. Please try again.',
                    timestamp: new Date().toISOString(),
                });
            } finally {
                setIsLoading(false);
            }
        }, [currentMode, selectedText, addMessage]
    );

    const setMode = useCallback((mode) => {
        if (mode === 'global' || mode === 'selection') {
            setCurrentMode(mode);
        }
    }, []);

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
        setMode,
        updateSelectedText,
    };
};

export default useChat;