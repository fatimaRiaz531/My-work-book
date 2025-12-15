/**
 * Embed script for RAG Chatbot Widget
 * This script allows easy integration of the chatbot into Docusaurus or other websites
 */

(function() {
  // Configuration
  const CONFIG = {
    apiUrl: window.RAG_CHATBOT_API_URL || 'http://localhost:8000/api/v1',
    containerId: 'rag-chatbot-container',
    widgetId: 'rag-chatbot-widget',
    buttonId: 'rag-chatbot-toggle'
  };

  // Create the chatbot container and button
  function initializeChatbot() {
    // Check if widget is already loaded
    if (document.getElementById(CONFIG.widgetId)) {
      return;
    }

    // Create toggle button
    const toggleButton = document.createElement('div');
    toggleButton.id = CONFIG.buttonId;
    toggleButton.innerHTML = `
      <div style="
        position: fixed;
        bottom: 20px;
        right: 20px;
        width: 60px;
        height: 60px;
        border-radius: 50%;
        background: #3b82f6;
        color: white;
        display: flex;
        align-items: center;
        justify-content: center;
        cursor: pointer;
        box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        z-index: 10000;
        font-size: 24px;
      " title="Open Book Assistant">
        <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
        </svg>
      </div>
    `;

    // Create widget container
    const widgetContainer = document.createElement('div');
    widgetContainer.id = CONFIG.widgetId;
    widgetContainer.innerHTML = `
      <div id="${CONFIG.containerId}" style="
        position: fixed;
        bottom: 90px;
        right: 20px;
        width: 400px;
        height: 500px;
        z-index: 10000;
        display: none;
        box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.1), 0 4px 6px -2px rgba(0, 0, 0, 0.05);
        border-radius: 0.5rem;
        overflow: hidden;
      "></div>
    `;

    // Add to document
    document.body.appendChild(toggleButton);
    document.body.appendChild(widgetContainer);

    // Toggle chatbot visibility
    let chatbotLoaded = false;
    toggleButton.addEventListener('click', function() {
      const container = document.getElementById(CONFIG.containerId);
      const isVisible = container.style.display !== 'none';

      if (isVisible) {
        container.style.display = 'none';
      } else {
        container.style.display = 'block';

        // Load the chatbot widget if not already loaded
        if (!chatbotLoaded) {
          loadChatbotWidget();
          chatbotLoaded = true;
        }
      }
    });

    // Close chatbot when clicking outside
    document.addEventListener('click', function(event) {
      const container = document.getElementById(CONFIG.containerId);
      const toggleButton = document.getElementById(CONFIG.buttonId);

      if (container.style.display !== 'none' &&
          !container.contains(event.target) &&
          !toggleButton.contains(event.target)) {
        container.style.display = 'none';
      }
    });
  }

  // Load the React chatbot widget
  function loadChatbotWidget() {
    // Check if React and ReactDOM are available
    if (typeof React === 'undefined' || typeof ReactDOM === 'undefined') {
      console.error('React and ReactDOM are required for the RAG Chatbot widget');
      return;
    }

    // Check if the widget is already loaded
    const existingWidget = document.getElementById('chatbot-root');
    if (existingWidget) {
      existingWidget.style.display = 'block';
      return;
    }

    // Create the container for the React app
    const container = document.getElementById(CONFIG.containerId);
    container.innerHTML = '<div id="chatbot-root" style="height: 100%;"></div>';

    // Import the ChatInterface component dynamically
    // In a production environment, you would bundle this with webpack
    // For now, we'll dynamically create a script tag to load the built widget
    const script = document.createElement('script');
    script.src = window.RAG_CHATBOT_WIDGET_URL || 'http://localhost:9000/chat-widget.js'; // Default to dev server
    script.async = true;

    script.onload = function() {
      // Wait for the module to be available
      if (window.RAGChatbot && window.RAGChatbot.renderChatInterface) {
        window.RAGChatbot.renderChatInterface('chatbot-root', CONFIG.apiUrl);
      } else {
        console.error('RAGChatbot module not loaded properly');
      }
    };

    script.onerror = function() {
      console.error('Failed to load RAG Chatbot widget');
      // Fallback to a simple HTML version
      createSimpleWidget(container);
    };

    document.head.appendChild(script);
  }

  // Create a simple fallback widget if React fails to load
  function createSimpleWidget(container) {
    const chatbotHTML = `
      <div id="chatbot-root" style="height: 100%; display: flex; flex-direction: column;">
        <div style="background-color: #1f2937; color: white; padding: 0.75rem; border-radius: 0.5rem 0.5rem 0 0; display: flex; justify-content: space-between; align-items: center;">
          <h3 style="font-weight: bold; margin: 0;">Book Assistant</h3>
          <button id="close-chatbot" style="background: #4b5563; color: white; border: none; border-radius: 0.25rem; padding: 0.25rem 0.5rem; cursor: pointer;">✕</button>
        </div>
        <div id="messages-container" style="flex: 1; overflow-y: auto; padding: 1rem; background-color: #f9fafb;">
          <div style="text-align: center; color: #6b7280; margin-top: 2.5rem;">
            <p>Ask me anything about the book!</p>
            <p style="font-size: 0.875rem; margin-top: 0.25rem;">Select text and ask questions about it specifically.</p>
          </div>
        </div>
        <div style="padding: 0.75rem; border-top: 1px solid #e5e7eb; background-color: #f9fafb;">
          <form id="chat-form" style="display: flex;">
            <input
              type="text"
              id="chat-input"
              placeholder="Ask about the book..."
              style="flex: 1; border: 1px solid #d1d5db; border-radius: 0.5rem 0 0 0.5rem; padding: 0.5rem 0.75rem; outline: none;"
            />
            <button
              type="submit"
              style="background-color: #3b82f6; color: white; border: none; border-radius: 0 0.5rem 0.5rem 0; padding: 0.5rem 1rem; cursor: pointer;"
            >
              Send
            </button>
          </form>
        </div>
      </div>
    `;

    container.innerHTML = chatbotHTML;

    // Add event listeners for the fallback widget
    document.getElementById('close-chatbot').addEventListener('click', function() {
      const widgetContainer = document.getElementById(CONFIG.containerId);
      widgetContainer.style.display = 'none';
    });

    document.getElementById('chat-form').addEventListener('submit', function(e) {
      e.preventDefault();
      const input = document.getElementById('chat-input');
      const message = input.value.trim();
      if (message) {
        // In a real implementation, this would call the API
        console.log('Sending message:', message);
        input.value = '';
      }
    });
  }

  // Initialize when DOM is loaded
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initializeChatbot);
  } else {
    initializeChatbot();
  }
})();