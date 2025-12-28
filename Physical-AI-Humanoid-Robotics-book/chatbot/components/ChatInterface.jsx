// chatbot/components/ChatInterface.jsx
import React, { useState, useRef, useEffect } from 'react';
import PropTypes from 'prop-types';
import ChatMessage from './ChatMessage';
import useChat from '../hooks/useChat';
import useTextSelection from '../hooks/useTextSelection';
import ModeSelector from './ModeSelector';
import '../styles/chat.css';

const ChatInterface = () => {
  const { messages, isLoading, error, currentMode, processQuery, setMode, updateSelectedText } = useChat();
  const { selectedText: currentSelectedText } = useTextSelection();
  const [inputValue, setInputValue] = useState('');
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Sync selected text with chat context
  useEffect(() => {
    updateSelectedText(currentSelectedText);
  }, [currentSelectedText, updateSelectedText]);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSubmit = async () => {
    if (!inputValue.trim() || isLoading) return;

    await processQuery(inputValue.trim());
    setInputValue('');
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault(); // 🚫 page reload stop
      handleSubmit();
    }
  };

  return (
    <div className="chatbot-container">
      <div className="chat-header">
        <h3 className="chat-header-title">AI Assistant</h3>
      </div>

      <div className="chat-history">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>Hello! I'm your AI assistant for this documentation.</p>
            <p>You can ask me questions about the entire book or about specific text you've selected.</p>
          </div>
        ) : (
          messages.map((message, index) => (
            <ChatMessage key={message.id || index} message={message} />
          ))
        )}

        {isLoading && (
          <div className="chat-message message-assistant">
            <span className="message-role">Assistant</span>
            <div className="loading-indicator">Thinking...</div>
          </div>
        )}

        {error && <div className="error-message">{error}</div>}

        <div ref={messagesEndRef} />
      </div>

      <div className="chat-input-area">
        <ModeSelector currentMode={currentMode} onModeChange={setMode} />

        {currentMode === 'selection' && currentSelectedText && (
          <div className="selected-text-indicator">
            <strong>Selected text:</strong>{' '}
            {currentSelectedText.substring(0, 100)}
            {currentSelectedText.length > 100 ? '...' : ''}
          </div>
        )}

        {/* 🚫 FORM REMOVED – PAGE RELOAD FIX */}
        <div className="chat-input-form">
          <textarea
            ref={inputRef}
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder={
              currentMode === 'selection' && currentSelectedText
                ? 'Ask a question about the selected text...'
                : 'Ask a question about the documentation...'
            }
            className="chat-input"
            rows={1}
            disabled={isLoading}
            maxLength={2000}
          />

          <button
            type="button" // 🔑 VERY IMPORTANT
            onClick={handleSubmit}
            disabled={!inputValue.trim() || isLoading}
            className="send-button"
          >
            {isLoading ? '...' : 'Send'}
          </button>
        </div>
      </div>
    </div>
  );
};

ChatInterface.propTypes = {};

export default ChatInterface;
