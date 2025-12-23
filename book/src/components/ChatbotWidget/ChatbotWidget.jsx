import React, { useState, useEffect } from 'react';
import './ChatbotWidget.css';

const ChatbotWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  // Configuration - use environment-based API URL for production
  // For Vercel deployment, this will be configured as an environment variable
  // Default to a known backend URL to work with proxy setups
  const API_BASE_URL = typeof process !== 'undefined' && process.env ? (process.env.REACT_APP_API_URL || 'http://localhost:8000/api/v1') : 'http://localhost:8000/api/v1';

  // Function to get selected text
  const getSelectedText = () => {
    const selection = window.getSelection();
    return selection.toString().trim();
  };

  // Function to handle sending a message
  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = inputValue.trim();
    const selectedText = getSelectedText();

    // Add user message to chat
    const userMsgObj = { role: 'user', content: userMessage, timestamp: new Date() };
    setMessages(prev => [...prev, userMsgObj]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Prepare the request
      let response;
      if (selectedText) {
        // Use selection-based endpoint
        response = await fetch(`${API_BASE_URL}/chat/selection`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            message: userMessage,
            selection: selectedText,
          }),
        });
      } else {
        // Use general chat endpoint
        response = await fetch(`${API_BASE_URL}/chat`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            message: userMessage,
          }),
        });
      }

      if (!response.ok) {
        throw new Error(`API error: ${response.status}`);
      }

      const data = await response.json();

      // Add bot response to chat
      const botMsgObj = {
        role: 'assistant',
        content: data.response,
        sources: data.sources || [],
        timestamp: new Date()
      };
      setMessages(prev => [...prev, botMsgObj]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMsg = {
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMsg]);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle Enter key press
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  // Function to render sources
  const renderSources = (sources) => {
    if (!sources || sources.length === 0) return null;

    return (
      <div className="sources">
        <h4>Sources:</h4>
        <ul>
          {sources.map((source, index) => (
            <li key={index}>
              {source.title || source.section || source.source_file}
            </li>
          ))}
        </ul>
      </div>
    );
  };

  return (
    <div className="chatbot-container">
      {isOpen ? (
        <div className="chatbot-window">
          <div className="chatbot-header">
            <h3>Book Assistant</h3>
            <button
              className="close-button"
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>

          <div className="chat-messages">
            {messages.length === 0 ? (
              <div className="welcome-message">
                <p>Hello! I'm your Book Assistant.</p>
                <p>You can ask me questions about the book content.</p>
                {getSelectedText() && (
                  <p><em>Selected text: "{getSelectedText().substring(0, 50)}..."</em></p>
                )}
              </div>
            ) : (
              messages.map((msg, index) => (
                <div
                  key={index}
                  className={`message ${msg.role === 'user' ? 'user-message' : 'bot-message'}`}
                >
                  <div className="message-content">
                    {msg.content}
                    {msg.sources && renderSources(msg.sources)}
                  </div>
                </div>
              ))
            )}
          </div>

          <div className="chat-input-area">
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask about the book..."
              disabled={isLoading}
              rows="2"
            />
            <button
              onClick={sendMessage}
              disabled={isLoading || !inputValue.trim()}
              className="send-button"
            >
              {isLoading ? 'Sending...' : 'Send'}
            </button>
          </div>
        </div>
      ) : (
        <button
          className="chatbot-toggle"
          onClick={() => setIsOpen(true)}
          aria-label="Open chatbot"
        >
          <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        </button>
      )}
    </div>
  );
};

export default ChatbotWidget;