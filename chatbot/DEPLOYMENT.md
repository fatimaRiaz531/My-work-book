# 🚀 RAG Chatbot Backend Deployment Guide

## Architecture Overview

```
Docusaurus Frontend (book/)
    ↓ (HTTP POST /query)
FastAPI Backend (chatbot/)
    ↓
Qdrant Cloud (Vector DB)
    ↓
OpenAI API (Embeddings + GPT-4)
    ↓
Response with Citations
```

## Prerequisites

- Python 3.9+
- OpenAI API key (https://platform.openai.com/api-keys)
- Qdrant Cloud account (https://cloud.qdrant.io/)
- Vercel/Railway account (for deployment)

## Local Setup (5 minutes)

### Step 1: Create Qdrant Cloud Cluster

1. Go to https://cloud.qdrant.io/
2. Sign up or login
3. Create new cluster: "robotics-textbook"
4. Get cluster URL and API key
5. Note the connection details

### Step 2: Configure Environment

```bash
# Go to chatbot directory
cd chatbot

# Copy .env.example to .env
cp .env.example .env

# Edit .env with your credentials
# - QDRANT_URL: from Qdrant Cloud
# - QDRANT_API_KEY: from Qdrant Cloud
# - OPENAI_API_KEY: from OpenAI platform
```

### Step 3: Install Dependencies

```bash
pip install -r requirements.txt
```

### Step 4: Index Content

```bash
# This will:
# 1. Read all 4 module files
# 2. Split into chunks
# 3. Generate embeddings
# 4. Upload to Qdrant

python index_content.py

# Expected output:
# ✅ INDEXING COMPLETE
#    Total chunks indexed: ~150-200
#    Collection: robotics-textbook
```

### Step 5: Run Locally

```bash
# Start FastAPI server
python -m uvicorn main:app --reload --host 0.0.0.0 --port 8000

# API docs will be at: http://localhost:8000/docs
# Test endpoint: POST http://localhost:8000/query
```

### Step 6: Test Chatbot

```bash
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
```

**Expected Response:**

```json
{
  "answer": "ROS 2 (Robot Operating System 2) is a middleware...",
  "sources": [
    {
      "module": "Module 1: ROS 2 Fundamentals",
      "section": "Introduction to ROS 2",
      "content_preview": "ROS 2 represents a fundamental paradigm shift...",
      "relevance_score": 0.87
    }
  ],
  "query_tokens": 45,
  "response_tokens": 120,
  "timestamp": "2025-12-07T10:30:00"
}
```

## Production Deployment (Vercel)

### Step 1: Create Vercel Project

```bash
# Go to project root
cd ..

# Vercel needs special setup for Python
# Create vercel.json
cat > vercel.json << 'EOF'
{
  "buildCommand": "pip install -r chatbot/requirements.txt",
  "outputDirectory": ".",
  "env": {
    "QDRANT_URL": "@qdrant_url",
    "QDRANT_API_KEY": "@qdrant_api_key",
    "OPENAI_API_KEY": "@openai_api_key"
  }
}
EOF
```

### Step 2: Create API Handler

```bash
# Create api/chatbot.py for Vercel
mkdir -p api
cp chatbot/main.py api/chatbot.py
```

### Step 3: Deploy to Vercel

```bash
# Install Vercel CLI
npm i -g vercel

# Deploy
vercel deploy

# Add environment variables in Vercel dashboard:
# - QDRANT_URL
# - QDRANT_API_KEY
# - OPENAI_API_KEY
```

## Production Deployment (Railway)

### Step 1: Deploy on Railway

```bash
# Install Railway CLI
npm i -g @railway/cli

# Login
railway login

# Initialize project
railway init

# Deploy
railway up

# Add environment variables:
railway variables add QDRANT_URL=...
railway variables add QDRANT_API_KEY=...
railway variables add OPENAI_API_KEY=...
```

### Step 2: Get Deployment URL

```bash
# Railway will provide a URL like:
# https://robotics-chatbot.railway.app

# Test:
curl -X POST "https://robotics-chatbot.railway.app/query" \
  -H "Content-Type: application/json" \
  -d '{"question": "How does reinforcement learning work?"}'
```

## Docusaurus Integration

### Step 1: Create React Component

File: `book/src/components/ChatBot.jsx`

```jsx
import React, { useState } from 'react';
import styles from './ChatBot.module.css';

export default function ChatBot() {
  const [question, setQuestion] = useState('');
  const [answer, setAnswer] = useState(null);
  const [loading, setLoading] = useState(false);

  const handleQuery = async () => {
    if (!question.trim()) return;

    setLoading(true);
    try {
      const response = await fetch(
        process.env.REACT_APP_CHATBOT_URL || 'http://localhost:8000',
        {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ question, top_k: 7 }),
        }
      );

      const data = await response.json();
      setAnswer(data);
    } catch (error) {
      console.error('Query failed:', error);
      setAnswer({ error: 'Failed to get response' });
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.chatbot}>
      <h3>📚 Ask About the Textbook</h3>
      <div className={styles.input_group}>
        <input
          type="text"
          value={question}
          onChange={(e) => setQuestion(e.target.value)}
          onKeyPress={(e) => e.key === 'Enter' && handleQuery()}
          placeholder="Ask a question..."
          disabled={loading}
        />
        <button onClick={handleQuery} disabled={loading}>
          {loading ? 'Loading...' : 'Ask'}
        </button>
      </div>

      {answer && (
        <div className={styles.response}>
          <p>{answer.answer}</p>
          {answer.sources && answer.sources.length > 0 && (
            <div className={styles.sources}>
              <h4>📖 Sources:</h4>
              {answer.sources.map((src, i) => (
                <div key={i} className={styles.source}>
                  <strong>{src.module}</strong> - {src.section}
                  <span className={styles.score}>
                    Score: {src.relevance_score}
                  </span>
                </div>
              ))}
            </div>
          )}
        </div>
      )}
    </div>
  );
}
```

### Step 2: Add to Docusaurus Sidebar

```jsx
// book/src/components/ChatBotWidget.jsx
import ChatBot from './ChatBot';

export default function ChatBotWidget() {
  return <ChatBot />;
}
```

### Step 3: Update docusaurus.config.js

```js
themeConfig: {
  customCss: [
    require.resolve('./src/css/custom.css'),
  ],
  navbar: {
    items: [
      // ... existing items
      {
        type: 'custom-chatbot',
        position: 'right',
      },
    ],
  },
}
```

## Cost Estimation

| Service        | Cost/Month  | Usage                            |
| -------------- | ----------- | -------------------------------- |
| Qdrant Cloud   | ~$10-50     | Vector DB with 150-200 points    |
| OpenAI API     | ~$5-20      | ~1000 queries × (~50 tokens avg) |
| Vercel/Railway | Free-$20    | Hobby/pro tier                   |
| **Total**      | **~$20-90** | Full production setup            |

## Monitoring & Logging

```bash
# View Vercel logs
vercel logs

# View Railway logs
railway logs

# Monitor API usage
# Check OpenAI dashboard: https://platform.openai.com/account/billing/overview
# Check Qdrant usage: https://cloud.qdrant.io/
```

## Troubleshooting

### Issue: "Collection not found"

```
Solution: Run index_content.py to create and populate collection
```

### Issue: "OpenAI API rate limit"

```
Solution:
- Add delays between requests
- Increase token limit in OpenAI dashboard
- Use GPT-3.5-turbo instead of GPT-4
```

### Issue: "Qdrant connection failed"

```
Solution:
- Check API key and URL in .env
- Verify Qdrant cluster is running
- Check network connectivity
```

### Issue: "CORS errors from Docusaurus"

```
Solution:
- Chatbot backend has CORS enabled
- Verify frontend URL is allowed
- Check browser console for specific error
```

## Next Steps

1. ✅ Deploy backend
2. ✅ Index content
3. ✅ Integrate with Docusaurus
4. 📊 Monitor usage and costs
5. 🔄 Iterate based on user feedback

## Support

- FastAPI docs: https://fastapi.tiangolo.com/
- Qdrant docs: https://qdrant.tech/documentation/
- OpenAI API: https://platform.openai.com/docs/

---

**Status:** 🟢 Ready for deployment  
**Time to deploy:** ~15 minutes  
**Cost to run:** ~$50-100/month
