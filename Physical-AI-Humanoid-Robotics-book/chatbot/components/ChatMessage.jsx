// chatbot/components/ChatMessage.jsx
import React from 'react';
import PropTypes from 'prop-types';

const ChatMessage = ({ message }) => {
  const { role, content, sources, timestamp, status } = message;

  // Format timestamp if available
  const formattedTime = timestamp ? new Date(timestamp).toLocaleTimeString() : '';

  return (
    <div className={`chat-message message-${role}`}>
      <span className="message-role">
        {role === 'user' ? 'You' : 'Assistant'}
        {formattedTime && <span className="message-time"> • {formattedTime}</span>}
      </span>
      <p className="message-content">{content}</p>

      {status === 'error' && (
        <div className="error-message">
          Sorry, there was an error processing your request. Please try again.
        </div>
      )}

      {sources && sources.length > 0 && (
        <div className="sources-container">
          <div className="sources-title">Sources:</div>
          {sources.map((source, index) => (
            <div key={index} className="source-item">
              {source.section_title && (
                <div className="source-title">{source.section_title}</div>
              )}
              {source.content_snippet && (
                <div className="source-content">{source.content_snippet}</div>
              )}
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

ChatMessage.propTypes = {
  message: PropTypes.shape({
    role: PropTypes.oneOf(['user', 'assistant']).isRequired,
    content: PropTypes.string.isRequired,
    sources: PropTypes.array,
    timestamp: PropTypes.string,
    status: PropTypes.oneOf(['pending', 'success', 'error']),
  }).isRequired,
};

export default ChatMessage;