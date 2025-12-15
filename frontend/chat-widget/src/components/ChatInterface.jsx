import React, { useState, useRef, useEffect } from 'react';
import Message from './Message';
import ApiClient from '../services/api-client';

const ChatInterface = ({ apiUrl }) => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  const apiClient = new ApiClient(apiUrl);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Function to get selected text
  const getSelectedText = () => {
    const selection = window.getSelection();
    return selection.toString().trim();
  };

  // Function to handle text selection
  const handleSelection = () => {
    const text = getSelectedText();
    if (text) {
      setSelectedText(text);
      // Focus the input and add a prompt for the user
      inputRef.current?.focus();
    }
  };

  // Add event listener for text selection
  useEffect(() => {
    const handleMouseUp = () => {
      handleSelection();
    };

    document.addEventListener('mouseup', handleMouseUp);
    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, []);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message to the chat
    const userMessage = { role: 'user', content: inputValue, isUser: true };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      let response;

      if (selectedText) {
        // Use selection-based endpoint
        response = await apiClient.chatWithSelection(inputValue, selectedText, sessionId);
        // Clear the selected text after using it
        setSelectedText('');
      } else {
        // Use global chat endpoint
        response = await apiClient.chat(inputValue, sessionId);
      }

      // Update session ID if new one was provided
      if (response.session_id && !sessionId) {
        setSessionId(response.session_id);
      }

      // Add assistant message to the chat
      const assistantMessage = {
        role: 'assistant',
        content: response.response,
        sources: response.sources,
        isUser: false
      };
      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        isUser: false
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleClearChat = () => {
    setMessages([]);
    setSessionId(null);
    setSelectedText('');
  };

  return (
    <div className="chat-container bg-white border border-gray-300 rounded-lg shadow-lg flex flex-col h-[500px] w-full max-w-md">
      {/* Chat header */}
      <div className="chat-header bg-gray-800 text-white p-3 rounded-t-lg flex justify-between items-center">
        <h3 className="font-bold">Book Assistant</h3>
        <button
          onClick={handleClearChat}
          className="text-sm bg-gray-700 hover:bg-gray-600 px-2 py-1 rounded"
        >
          Clear
        </button>
      </div>

      {/* Selected text indicator */}
      {selectedText && (
        <div className="selected-text-indicator bg-yellow-100 p-2 text-sm border-b">
          <span className="font-medium">Using selection:</span> "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"
        </div>
      )}

      {/* Messages container */}
      <div className="messages-container flex-1 overflow-y-auto p-3">
        {messages.length === 0 ? (
          <div className="welcome-message text-center text-gray-500 mt-10">
            <p>Ask me anything about the book!</p>
            <p className="text-sm mt-2">Select text and ask questions about it specifically.</p>
          </div>
        ) : (
          messages.map((msg, index) => (
            <Message
              key={index}
              message={msg.content}
              isUser={msg.isUser}
            />
          ))
        )}
        {isLoading && (
          <div className="message message-assistant bg-gray-200 text-gray-800 max-w-[80%] p-3 rounded-lg mb-3">
            <div className="typing-indicator">
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      {/* Input area */}
      <form onSubmit={handleSubmit} className="input-area p-3 border-t bg-gray-50">
        <div className="flex">
          <input
            ref={inputRef}
            type="text"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            placeholder={selectedText ? "Ask about selected text..." : "Ask about the book..."}
            className="flex-1 border border-gray-300 rounded-l-lg px-3 py-2 focus:outline-none focus:ring-2 focus:ring-blue-500"
            disabled={isLoading}
          />
          <button
            type="submit"
            disabled={!inputValue.trim() || isLoading}
            className={`bg-blue-500 text-white px-4 py-2 rounded-r-lg ${(!inputValue.trim() || isLoading) ? 'opacity-50 cursor-not-allowed' : 'hover:bg-blue-600'}`}
          >
            Send
          </button>
        </div>
      </form>
    </div>
  );
};

export default ChatInterface;