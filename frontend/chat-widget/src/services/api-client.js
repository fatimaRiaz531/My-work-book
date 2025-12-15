/**
 * API client for the RAG Chatbot
 */
class ApiClient {
  constructor(baseURL) {
    this.baseURL = baseURL || process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000/api/v1';
  }

  /**
   * Send a message to the global chat endpoint
   */
  async chat(message, sessionId = null, history = []) {
    const response = await fetch(`${this.baseURL}/chat`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        message,
        session_id: sessionId,
        history
      })
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return response.json();
  }

  /**
   * Send a message with text selection to the selection chat endpoint
   */
  async chatWithSelection(message, selection, sessionId = null, history = []) {
    const response = await fetch(`${this.baseURL}/chat/selection`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        message,
        selection,
        session_id: sessionId,
        history
      })
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return response.json();
  }

  /**
   * Check the health of the API
   */
  async healthCheck() {
    const response = await fetch(`${this.baseURL}/health`);

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return response.json();
  }
}

export default ApiClient;