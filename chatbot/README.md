# 🤖 RAG Chatbot Quick Start

## What is This?

A **Retrieval-Augmented Generation (RAG)** chatbot that:

1. Takes user questions
2. Searches the textbook for relevant content
3. Uses GPT-4 to generate answers
4. Cites sources automatically

## 30-Second Setup

```bash
# 1. Get credentials
# - Qdrant: https://cloud.qdrant.io/
# - OpenAI: https://platform.openai.com/api-keys

# 2. Setup
cd chatbot
cp .env.example .env
# Edit .env with your credentials

# 3. Install
pip install -r requirements.txt

# 4. Index content
python index_content.py
# This takes ~2-5 minutes

# 5. Run
python -m uvicorn main:app --reload

# 6. Test at http://localhost:8000/docs
```

## API Usage

### Query Endpoint

```bash
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "How does ROS 2 work?",
    "top_k": 7,
    "threshold": 0.5
  }'
```

### Response Example

```json
{
  "answer": "ROS 2 (Robot Operating System 2) is a middleware that connects different software components (nodes) running on robots. It uses DDS (Data Distribution Service) as its transport layer, enabling pub-sub and request-response communication patterns [Source: Module 1 - Introduction to ROS 2]...",
  "sources": [
    {
      "module": "Module 1: ROS 2 Fundamentals",
      "section": "Introduction to ROS 2",
      "content_preview": "Robot Operating System 2 (ROS 2) represents a fundamental paradigm shift...",
      "relevance_score": 0.92
    }
  ],
  "query_tokens": 45,
  "response_tokens": 187,
  "timestamp": "2025-12-07T10:30:00"
}
```

## Key Features

✅ **Semantic Search** - Finds relevant content using embeddings  
✅ **Citation Tracking** - Shows which chapter/section answered your question  
✅ **Smart Chunking** - Breaks content into ~500-word chunks with overlap  
✅ **Fast Responses** - <2 seconds end-to-end (with good network)  
✅ **Scalable** - Cloud-based Qdrant handles millions of vectors  
✅ **Accurate** - GPT-4 generates coherent, contextual answers

## File Structure

```
chatbot/
├── main.py              # FastAPI app (endpoints, generation)
├── index_content.py     # Content indexing script
├── requirements.txt     # Python dependencies
├── .env.example         # Template for environment variables
├── DEPLOYMENT.md        # Full deployment guide
└── README.md           # This file
```

## How It Works (Under the Hood)

```
User Question: "What is ROS 2?"
    ↓
[STEP 1] Convert to embedding (OpenAI API)
    → "What is ROS 2?" → [0.124, -0.532, 0.891, ...]
    ↓
[STEP 2] Search Qdrant for similar embeddings
    → Find top 7 most similar chunks
    → ROS 2 intro section: score 0.92
    → ROS 2 concepts section: score 0.87
    → DDS middleware section: score 0.84
    → Nav2 planning section: score 0.62
    → Gazebo simulation section: score 0.51
    → Additional relevant section 6: score 0.48
    → Additional relevant section 7: score 0.45
    ↓
[STEP 3] Build context from results
    → Combine top 5 chunks into single context
    ↓
[STEP 4] Generate answer with GPT-4
    → "Given this context: [top 5 chunks],
        answer this question: What is ROS 2?"
    → GPT-4: "ROS 2 is a middleware..."
    ↓
[STEP 5] Format response
    → Answer + citations + source links
    ↓
User gets: Answer with automatic citations ✨
```

## Example Queries

Try these questions after setting up:

### Module 1 (ROS 2)

- "How do ROS 2 nodes communicate?"
- "What is a ROS 2 topic vs service?"
- "How to write a ROS 2 Python node?"
- "What is URDF and why do we need it?"

### Module 2 (Gazebo & Unity)

- "How does physics simulation work in Gazebo?"
- "What's the difference between URDF and SDF?"
- "How to simulate sensors like LiDAR?"
- "Why use digital twins?"

### Module 3 (NVIDIA Isaac)

- "What is reinforcement learning for robots?"
- "How does sim-to-real transfer work?"
- "What is domain randomization?"
- "How to train a policy in Isaac Sim?"

### Module 4 (VLA)

- "What are Vision-Language-Action models?"
- "How does Whisper speech recognition work?"
- "How to integrate LLMs with ROS 2?"
- "What is multi-modal perception?"

## Costs

| Item                      | Cost              |
| ------------------------- | ----------------- |
| Qdrant Cloud (small)      | $10-20/month      |
| OpenAI API (1000 queries) | $5-15/month       |
| Vercel/Railway hosting    | Free-$20/month    |
| **Total**                 | **~$25-50/month** |

## Scaling Tips

1. **Cache responses** - Store common Q&A
2. **Use GPT-3.5-turbo** - Faster, cheaper than GPT-4
3. **Batch embeddings** - Process multiple at once
4. **Monitor costs** - Set OpenAI spending limits
5. **Optimize chunks** - Test different chunk sizes

## Advanced Features (Future)

- [ ] Multi-language support (Hindi, Urdu, Chinese)
- [ ] Follow-up questions (conversation history)
- [ ] User feedback (thumbs up/down)
- [ ] Conversation analytics
- [ ] Fine-tuned LLM for domain
- [ ] Local LLM option (Ollama)
- [ ] Vector DB alternatives (Pinecone, Weaviate)

## Troubleshooting

**Q: Chatbot gives wrong answers?**
A: The search might be finding wrong chunks. Try rephrasing your question.

**Q: API calls are slow?**
A: Check internet speed. GPT-4 inference takes ~0.5-1s. Use GPT-3.5-turbo for speed.

**Q: Getting rate limited?**
A: Contact OpenAI to increase rate limits or use GPT-3.5-turbo.

**Q: Vector DB is empty?**
A: Run `python index_content.py` to populate it.

## Next Steps

1. ✅ Setup locally and test
2. ✅ Deploy to Vercel/Railway
3. ✅ Integrate React component in Docusaurus
4. ✅ Monitor usage and iterate
5. ✅ Add follow-up questions feature

---

**Need help?** Check DEPLOYMENT.md for detailed setup guide.

**Built with:** FastAPI • Qdrant • OpenAI GPT-4 • Python 3.9+
