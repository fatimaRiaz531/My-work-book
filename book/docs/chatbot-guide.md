---
sidebar_position: 7
---

# Chatbot Guide

Welcome to the embedded RAG chatbot! This guide explains how to use the intelligent chatbot integrated into this textbook.

## What is the RAG Chatbot?

**RAG** stands for **Retrieval-Augmented Generation**. Our chatbot:

1. **Retrieves** relevant passages from the book
2. **Reads** the content to understand context
3. **Generates** answers directly from the book material
4. **Cites** specific chapters and sections

This means **all answers are grounded in the textbook**—no hallucinations or made-up information.

---

## How to Use the Chatbot

### 1. Ask a Question

Click the **💬 Chat** button in the sidebar or the floating button on any page.

Type your question:

```
"What is ROS 2?"
"How do I create a ROS 2 node?"
"What are the hardware requirements?"
```

### 2. Get an Answer

The chatbot responds with:

- **Direct answer** from the relevant section
- **Citation** (e.g., "See Module 1, Section: Introduction to ROS 2")
- **Code examples** if applicable
- **Links** to related sections

### 3. Ask a Follow-Up

Continue the conversation:

```
Original question: "What is ROS 2?"
Follow-up: "How does it compare to ROS 1?"
Follow-up: "Show me a code example"
```

---

## Advanced Features

### Select Text to Query

You can select any text passage in the book and ask the chatbot about it:

1. **Highlight** a passage you want to ask about
2. Click **"Ask Chatbot"** (appears near your selection)
3. Chatbot will answer based on that specific passage

Example:

> "URDF is the Unified Robot Description Format used to describe robot structure."

Ask: "Explain this passage in simpler terms"

### Code Example Generation

Ask for code examples:

```
"Give me a Python ROS 2 publisher example"
"How do I use Gazebo from ROS 2?"
"Show me a service server in Python"
```

The chatbot responds with:

- Complete, runnable code
- Explanations of key lines
- Links to full module sections

### Hardware Questions

Ask about hardware setups:

```
"What is the minimum hardware needed?"
"Can I run this on a Raspberry Pi?"
"What GPU do I need for Isaac Sim?"
```

---

## Chatbot Capabilities

| Question Type   | Example                  | Chatbot Can Do               |
| --------------- | ------------------------ | ---------------------------- |
| Concepts        | "What is VSLAM?"         | ✅ Explain with references   |
| Code            | "Show a ROS 2 publisher" | ✅ Provide working code      |
| Hardware        | "What is a Jetson?"      | ✅ Link to hardware section  |
| Troubleshooting | "My code won't build"    | ⚠️ Limited (book-based only) |
| Philosophy      | "Why robotics matters"   | ✅ Explain with sources      |
| Current events  | "Latest in AI 2025"      | ❌ No (knowledge cutoff)     |

---

## Tips for Better Answers

### Do:

- **Be specific:** "How do I publish sensor data in ROS 2?" (better than "Tell me about ROS 2")
- **Ask in context:** Use chatbot on the relevant module page
- **Follow up:** Ask for clarification or deeper dives
- **Reference sections:** "In Module 3, what does sim-to-real transfer mean?"

### Don't:

- **Ask outside scope:** The chatbot answers from this book only
- **Expect real-time info:** Book knowledge has a publication date
- **Expect custom solutions:** Chatbot provides book content, not debugging for your code

---

## Privacy & Data

- **Your questions** are sent to OpenAI/our backend for processing
- **Book content** is vectorized and stored in Qdrant Cloud (encrypted)
- **Conversation history** is stored in your browser (not on servers)
- **No data sharing** with third parties

To clear history: Open DevTools → Application → Clear Local Storage

---

## Troubleshooting

### Chatbot Not Responding

1. Check your internet connection
2. Verify API keys are configured
3. Try refreshing the page
4. Check browser console for errors

### Slow Responses

- Large queries take longer (normal)
- Try simpler, more focused questions
- If consistently slow, report an issue

### Answers Seem Wrong

- Chatbot learns from book content—answers may be incomplete
- **Always verify** with the textbook directly
- Report inaccuracies to authors

---

## Feedback

Help improve the chatbot:

- **"This was helpful"** → Feedback appreciated ✅
- **"This was confusing"** → Click to report issue
- **"I have a suggestion"** → Submit via feedback form

---

## FAQ

**Q: Can the chatbot answer questions not in the book?**  
A: No—it only answers from this textbook content.

**Q: Is the chatbot always accurate?**  
A: Very high accuracy (facts from the book), but not perfect. Always double-check important claims.

**Q: Can I use chatbot responses in my work?**  
A: Yes! Just cite the textbook. Example: "(Author Name, 2025, Module 1)"

**Q: How does RAG work?**  
A: The chatbot converts your question to vectors, searches the book's vector database (Qdrant), retrieves relevant sections, and generates an answer using GPT-4.

---

## Related Resources

- **Full Textbook:** Navigate chapters in the sidebar
- **References:** See all 60+ sources in the References page
- **Code Examples:** Each module includes runnable code
- **Hardware Guide:** See Appendix A for setup instructions

---

**Happy learning! Questions? Use the chatbot! 🤖**

**Last Updated:** December 7, 2025
