# Quickstart Guide: Docusaurus Embedded RAG Chatbot

## Prerequisites

- Node.js 18+ and npm/yarn
- Docusaurus project set up
- Backend API endpoint available for RAG queries

## Environment Setup

1. **Set up environment variables (.env.example file):**
   ```env
   REACT_APP_API_BASE_URL=your_backend_api_base_url
   REACT_APP_API_TIMEOUT=30000
   ```

2. **Install frontend dependencies:**
   ```bash
   cd your-docusaurus-project
   npm install
   ```

## Frontend Integration

1. **Create the chatbot component directory structure:**
   ```bash
   mkdir -p chatbot/components chatbot/hooks chatbot/services chatbot/styles
   ```

2. **Add the chatbot component to your Docusaurus layout:**
   ```jsx
   // src/components/DocusaurusChatbot.jsx
   import React, { useState } from 'react';
   import ChatInterface from '@site/chatbot/components/ChatInterface';

   export default function DocusaurusChatbot() {
     const [isVisible, setIsVisible] = useState(true);

     return (
       <div className="chatbot-container">
         {isVisible && <ChatInterface />}
       </div>
     );
   }
   ```

3. **Include the chatbot in your Docusaurus pages:**
   ```jsx
   // In your Docusaurus pages or layout
   import DocusaurusChatbot from '@site/src/components/DocusaurusChatbot';

   // Add to page layout
   <DocusaurusChatbot />
   ```

## Basic Usage

### Global Query Mode
```javascript
// Using the API client to send a global query
import { queryGlobal } from './services/apiClient';

const response = await queryGlobal({
  query: "What are the main components of a robot?",
  sessionId: "session-123"
});
```

### Selection Query Mode
```javascript
// Using the API client to send a selection-based query
import { querySelection } from './services/apiClient';

const response = await querySelection({
  query: "Explain this concept further",
  selectedText: "A robot typically consists of sensors, actuators, and a control system.",
  sessionId: "session-123"
});
```

## Development Workflow

1. Start Docusaurus: `npm run start`
2. Make changes to components in `chatbot/` directory
3. Test the chat interface functionality
4. Verify API communication with backend service

## API Configuration

The chatbot expects the following backend API endpoints:
- `POST /query` - For global book queries
- `POST /query/selection` - For selected text queries

The API should return responses in the format specified in the contract documentation.