// chatbot/services/utils.js
import { v4 as uuidv4 } from 'uuid';
import { getApiConfig } from './apiClient';

// ChatSession data structure
export class ChatSession {
  constructor(id, initialMode = 'global') {
    this.id = id || uuidv4();
    this.messages = [];
    this.currentMode = initialMode;
    this.createdAt = new Date().toISOString();
    this.updatedAt = new Date().toISOString();
  }

  // Add a message to the session
  addMessage(message) {
    if (!this.isValidMessage(message)) {
      throw new Error('Invalid message format');
    }

    this.messages.push({
      ...message,
      id: message.id || uuidv4(),
      timestamp: message.timestamp || new Date().toISOString(),
    });

    this.updatedAt = new Date().toISOString();
    return this;
  }

  // Validate message format
  isValidMessage(message) {
    return (
      message &&
      typeof message === 'object' &&
      ['user', 'assistant'].includes(message.role) &&
      typeof message.content === 'string' &&
      message.content.trim().length > 0
    );
  }

  // Get messages by role
  getMessagesByRole(role) {
    return this.messages.filter(msg => msg.role === role);
  }

  // Get the last message
  getLastMessage() {
    return this.messages.length > 0 ? this.messages[this.messages.length - 1] : null;
  }

  // Clear all messages
  clearMessages() {
    this.messages = [];
    this.updatedAt = new Date().toISOString();
    return this;
  }

  // Update the current mode
  updateMode(mode) {
    if (['global', 'selection'].includes(mode)) {
      this.currentMode = mode;
      this.updatedAt = new Date().toISOString();
      return this;
    } else {
      throw new Error('Invalid mode. Must be "global" or "selection"');
    }
  }

  // Get session summary
  getSummary() {
    return {
      id: this.id,
      messageCount: this.messages.length,
      userMessageCount: this.getMessagesByRole('user').length,
      assistantMessageCount: this.getMessagesByRole('assistant').length,
      currentMode: this.currentMode,
      createdAt: this.createdAt,
      updatedAt: this.updatedAt,
    };
  }
}

// Validation functions
export const validateQueryRequest = (request) => {
  if (!request || typeof request !== 'object') {
    throw new Error('Query request must be an object');
  }

  if (!request.query_text || typeof request.query_text !== 'string') {
    throw new Error('Query text is required and must be a string');
  }

  if (request.query_text.length < 1 || request.query_text.length > 2000) {
    throw new Error('Query text must be between 1 and 2000 characters');
  }

  if (!request.query_mode || !['global', 'selection'].includes(request.query_mode)) {
    throw new Error('Query mode is required and must be either "global" or "selection"');
  }

  if (request.query_mode === 'selection' && (!request.selected_text || typeof request.selected_text !== 'string')) {
    throw new Error('Selected text is required for selection mode');
  }

  if (request.query_mode === 'selection' && request.selected_text && request.selected_text.length > 5000) {
    throw new Error('Selected text must be 5000 characters or less');
  }

  return true;
};

export const validateChatMessage = (message) => {
  if (!message || typeof message !== 'object') {
    throw new Error('Message must be an object');
  }

  if (!message.role || !['user', 'assistant'].includes(message.role)) {
    throw new Error('Message role must be either "user" or "assistant"');
  }

  if (!message.content || typeof message.content !== 'string' || message.content.trim().length === 0) {
    throw new Error('Message content is required and must be a non-empty string');
  }

  if (message.sources && !Array.isArray(message.sources)) {
    throw new Error('Sources must be an array if provided');
  }

  return true;
};

// Create a new chat session
export const createChatSession = (initialMode = 'global') => {
  return new ChatSession(uuidv4(), initialMode);
};

// Generate a unique ID
export const generateId = () => uuidv4();

// Export API configuration
export { getApiConfig };