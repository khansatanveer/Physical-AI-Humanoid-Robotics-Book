// src/components/DocusaurusChatbot.jsx
// This component adds the chatbot UI to the page without interfering with content
import React, { useEffect } from 'react';

const DocusaurusChatbot = () => {
  useEffect(() => {
    // Create the chatbot launcher button dynamically
    const button = document.createElement('button');
    button.className = 'chatbot-launcher';
    button.setAttribute('aria-label', 'Open AI Assistant');
    button.style.position = 'fixed';
    button.style.bottom = '20px';
    button.style.right = '20px';
    button.style.zIndex = '2147483647'; // Maximum z-index
    button.style.width = '60px';
    button.style.height = '60px';
    button.style.borderRadius = '50%';
    button.style.backgroundColor = '#2563eb';
    button.style.color = 'white';
    button.style.border = 'none';
    button.style.cursor = 'pointer';
    button.style.fontSize = '20px';
    button.style.display = 'flex';
    button.style.alignItems = 'center';
    button.style.justifyContent = 'center';
    button.style.boxShadow = '0 4px 12px rgba(0, 0, 0, 0.15)';
    button.style.transition = 'all 0.2s ease';

    // Create the SVG icon
    const svgNS = "http://www.w3.org/2000/svg";
    const svg = document.createElementNS(svgNS, 'svg');
    svg.setAttribute('width', '24');
    svg.setAttribute('height', '24');
    svg.setAttribute('viewBox', '0 0 24 24');
    svg.setAttribute('fill', 'none');
    svg.setAttribute('stroke', 'currentColor');
    svg.setAttribute('strokeWidth', '2');
    svg.setAttribute('strokeLinecap', 'round');
    svg.setAttribute('strokeLinejoin', 'round');

    const path = document.createElementNS(svgNS, 'path');
    path.setAttribute('d', 'M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z');

    svg.appendChild(path);
    button.appendChild(svg);

    // Create the chat interface container
    const chatContainer = document.createElement('div');
    chatContainer.id = 'docusaurus-chatbot-interface';
    chatContainer.style.position = 'fixed';
    chatContainer.style.bottom = '20px';
    chatContainer.style.right = '20px';
    chatContainer.style.zIndex = '2147483646';
    chatContainer.style.display = 'none'; // Hidden by default

    // Add the chatbot styles to the page
    const style = document.createElement('style');
    style.textContent = `
      .chatbot-container {
        position: relative;
        width: 350px;
        max-width: 90vw;
        height: 500px;
        max-height: 70vh;
        display: flex;
        flex-direction: column;
        border-radius: 8px;
        box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
        background-color: white;
        overflow: hidden;
        font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
        z-index: inherit;
      }

      .chat-header {
        background-color: #2563eb;
        color: white;
        padding: 12px 16px;
        font-weight: 600;
        display: flex;
        justify-content: space-between;
        align-items: center;
      }

      .chat-header-title {
        font-size: 16px;
        margin: 0;
      }

      .chat-header-close {
        background: none;
        border: none;
        color: white;
        cursor: pointer;
        font-size: 18px;
        padding: 0;
        width: 24px;
        height: 24px;
        display: flex;
        align-items: center;
        justify-content: center;
      }

      .chat-history {
        flex: 1;
        overflow-y: auto;
        padding: 16px;
        background-color: #f9fafb;
        display: flex;
        flex-direction: column;
        gap: 12px;
      }

      .chat-message {
        max-width: 85%;
        padding: 10px 12px;
        border-radius: 18px;
        line-height: 1.4;
        position: relative;
      }

      .message-user {
        align-self: flex-end;
        background-color: #2563eb;
        color: white;
        border-bottom-right-radius: 4px;
      }

      .message-assistant {
        align-self: flex-start;
        background-color: #e2e8f0;
        color: #1e293b;
        border-bottom-left-radius: 4px;
      }

      .message-role {
        font-size: 12px;
        font-weight: 600;
        margin-bottom: 4px;
        display: block;
      }

      .message-content {
        margin: 0;
      }

      .sources-container {
        margin-top: 8px;
        padding-top: 8px;
        border-top: 1px solid #cbd5e1;
        font-size: 12px;
      }

      .source-item {
        margin-bottom: 4px;
        padding: 4px 0;
      }

      .source-title {
        font-weight: 600;
        color: #0f172a;
      }

      .source-content {
        color: #64748b;
        font-size: 11px;
        margin-top: 2px;
      }

      .chat-input-area {
        padding: 12px;
        background-color: white;
        border-top: 1px solid #e2e8f0;
        display: flex;
        flex-direction: column;
        gap: 8px;
      }

      .mode-selector {
        display: flex;
        gap: 8px;
        margin-bottom: 8px;
      }

      .mode-button {
        flex: 1;
        padding: 6px 8px;
        border: 1px solid #cbd5e1;
        background-color: white;
        border-radius: 4px;
        font-size: 12px;
        cursor: pointer;
      }

      .mode-button.active {
        background-color: #dbeafe;
        border-color: #2563eb;
        color: #2563eb;
      }

      .mode-button:hover {
        background-color: #f1f5f9;
      }

      .chat-input-form {
        display: flex;
        gap: 8px;
      }

      .chat-input {
        flex: 1;
        padding: 10px 12px;
        border: 1px solid #cbd5e1;
        border-radius: 4px;
        font-size: 14px;
        resize: none;
        min-height: 40px;
        max-height: 100px;
        overflow-y: auto;
      }

      .chat-input:focus {
        outline: none;
        border-color: #2563eb;
        box-shadow: 0 0 0 2px rgba(37, 99, 235, 0.2);
      }

      .send-button {
        padding: 10px 16px;
        background-color: #2563eb;
        color: white;
        border: none;
        border-radius: 4px;
        cursor: pointer;
        font-size: 14px;
        font-weight: 500;
      }

      .send-button:hover {
        background-color: #1d4ed8;
      }

      .send-button:disabled {
        background-color: #94a3b8;
        cursor: not-allowed;
      }

      .loading-indicator {
        display: flex;
        align-items: center;
        gap: 8px;
        padding: 10px 12px;
        color: #64748b;
        font-size: 14px;
      }

      .error-message {
        background-color: #fee2e2;
        color: #dc2626;
        padding: 10px 12px;
        border-radius: 4px;
        font-size: 14px;
        margin: 4px 0;
      }

      .selected-text-indicator {
        background-color: #fffbeb;
        color: #854d0e;
        padding: 8px 12px;
        border-radius: 4px;
        font-size: 12px;
        border: 1px solid #f59e0b;
        margin-bottom: 8px;
      }

      /* Responsive design */
      @media (max-width: 768px) {
        .chatbot-container {
          width: 95vw;
          height: 60vh;
          bottom: 10px;
          right: 10px;
        }

        .chat-message {
          max-width: 90%;
        }
      }
    `;
    document.head.appendChild(style);

    // Add the button and container to the body
    document.body.appendChild(button);
    document.body.appendChild(chatContainer);

    // Add click handler to the button
    let isChatOpen = false;

    const toggleChat = () => {
      isChatOpen = !isChatOpen;

      if (isChatOpen) {
        // Create the chat interface HTML dynamically
        chatContainer.innerHTML = `
          <div class="chatbot-container">
            <div class="chat-header">
              <h3 class="chat-header-title">AI Assistant</h3>
              <button class="chat-header-close" id="chatbot-close-btn">×</button>
            </div>

            <div class="chat-history" id="chatbot-history">
              <div class="welcome-message">
                <p>Hello! I'm your AI assistant for this documentation.</p>
                <p>You can ask me questions about the entire book or about specific text you've selected.</p>
              </div>
            </div>

            <div class="chat-input-area">
              <div class="mode-selector">
                <button class="mode-button active" id="mode-global">Global Book</button>
                <button class="mode-button" id="mode-selection">Selected Text</button>
              </div>

              <form class="chat-input-form" id="chatbot-form">
                <textarea
                  class="chat-input"
                  id="chatbot-input"
                  placeholder="Ask a question about the documentation..."
                  rows="1"
                  maxlength="2000"
                ></textarea>
                <button type="submit" class="send-button" id="chatbot-send-btn">Send</button>
              </form>
            </div>
          </div>
        `;

        // Show the chat container
        chatContainer.style.display = 'block';

        // Add event listeners to the dynamically created elements
        document.getElementById('chatbot-close-btn').onclick = () => {
          chatContainer.style.display = 'none';
          isChatOpen = false;
        };

        // For now, we'll just show a demo interface. In a real implementation,
        // you would integrate with the React components here.

        // Update button text to show it's active
        button.style.transform = 'scale(0.95)';
      } else {
        // Hide the chat container
        chatContainer.style.display = 'none';
        button.style.transform = 'scale(1)';
      }
    };

    button.onclick = toggleChat;

    // Cleanup function
    return () => {
      if (button && button.parentNode) {
        button.parentNode.removeChild(button);
      }
      if (chatContainer && chatContainer.parentNode) {
        chatContainer.parentNode.removeChild(chatContainer);
      }
      if (style && style.parentNode) {
        style.parentNode.removeChild(style);
      }
    };
  }, []);

  // This component doesn't render anything itself
  return null;
};

export default DocusaurusChatbot;