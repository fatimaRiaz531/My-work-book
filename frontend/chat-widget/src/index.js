import React from 'react';
import ReactDOM from 'react-dom/client';
import ChatInterface from './components/ChatInterface';

// Export the ChatInterface component for use in other applications
export { default as ChatInterface } from './components/ChatInterface';

// Also provide a function to render the component to a DOM element
export const renderChatInterface = (containerId, apiUrl) => {
  const container = document.getElementById(containerId);
  if (!container) {
    console.error(`Container with id "${containerId}" not found`);
    return;
  }

  const root = ReactDOM.createRoot(container);
  root.render(<ChatInterface apiUrl={apiUrl} />);
};

// For backward compatibility with older React versions
export const renderChatInterfaceLegacy = (containerId, apiUrl) => {
  const container = document.getElementById(containerId);
  if (!container) {
    console.error(`Container with id "${containerId}" not found`);
    return;
  }

  ReactDOM.render(<ChatInterface apiUrl={apiUrl} />, container);
};