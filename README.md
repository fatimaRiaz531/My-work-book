# Physical AI & Humanoid Robotics Course Book

This project implements a comprehensive textbook for teaching Physical AI & Humanoid Robotics with integrated AI features including a RAG chatbot, user authentication, personalization, and Urdu translation capabilities.

## 🚀 Features

- **Docusaurus-based Book**: Interactive textbook for Physical AI & Humanoid Robotics course
- **RAG Chatbot**: AI-powered assistant that answers questions about book content
- **User Authentication**: Sign up/sign in with background information collection
- **Content Personalization**: Personalized content based on user's software/hardware background
- **Urdu Translation**: Translate content to Urdu with one click
- **Reusable Intelligence**: Subagents for translation and personalization tasks

## 📚 Course Content

The book covers:
- Module 1: The Robotic Nervous System (ROS 2)
- Module 2: The Digital Twin (Gazebo & Unity)
- Module 3: The AI-Robot Brain (NVIDIA Isaac™)
- Module 4: Vision-Language-Action (VLA)

## 🛠️ Tech Stack

- **Frontend**: Docusaurus v3, React
- **Backend**: FastAPI, Python
- **Database**: PostgreSQL (Neon), Qdrant (Vector DB)
- **Authentication**: Custom JWT-based auth
- **AI/ML**: OpenAI, RAG (Retrieval-Augmented Generation)
- **Deployment**: Vercel (Frontend), Vercel/Cloud (Backend)

## 📁 Project Structure

```
├── book/                 # Docusaurus frontend
│   ├── docs/            # Course content
│   ├── src/             # React components
│   │   ├── components/  # Chatbot, Auth, Personalization
│   │   ├── lib/         # Subagents, Auth, Translation
│   │   └── theme/       # Layout with chatbot integration
│   └── docusaurus.config.js
├── backend/              # FastAPI backend
│   ├── src/
│   │   ├── api/         # API routers (chat, auth, health)
│   │   ├── models/      # Database models
│   │   └── services/    # RAG services
│   └── api.py           # Vercel-compatible entry point
├── vercel.json          # Frontend deployment config
└── vercel.backend.json  # Backend deployment config
```

## 🔧 Setup & Development

### Prerequisites
- Node.js 20+
- Python 3.10+
- PostgreSQL database
- Qdrant vector database

### Installation

```bash
# Install dependencies
npm run setup

# Install backend dependencies
cd backend
pip install -r requirements.txt
```

### Environment Variables

Create `.env` files in both directories:

**book/.env:**
```
REACT_APP_API_URL=https://your-backend-url.vercel.app/api/v1
```

**backend/.env:**
```
DATABASE_URL=postgresql://user:password@host:port/dbname
QDRANT_URL=https://your-cluster.europe-west3-4.gcp.cloud.qdrant.io:6333
QDRANT_API_KEY=your-api-key
QDRANT_COLLECTION_NAME=book_content
OPENAI_API_KEY=your-openai-key
```

### Development

```bash
# Start both frontend and backend
npm run dev

# Or start separately
npm run dev:book    # Frontend only
npm run dev:api     # Backend only
```

## 🚀 Deployment

### Option 1: Separate Deployments (Recommended)

1. **Deploy Backend to Vercel**:
   ```bash
   vercel backend/api.py --prod
   ```

2. **Deploy Frontend to Vercel/GitHub Pages**:
   ```bash
   cd book
   npm run build
   vercel --prod
   # OR for GitHub Pages
   npm run deploy
   ```

### Option 2: Environment Variables

For the frontend to work with deployed backend, set the environment variable:
- In Vercel: Add `REACT_APP_API_URL` with your backend API URL
- For GitHub Pages: Build with the correct API URL

## 🤖 AI Features

### RAG Chatbot
- Ask questions about book content
- Use text selection for context-specific answers
- Powered by OpenAI and vector search

### Authentication & Personalization
1. **Sign Up**: Provide software/hardware background
2. **Personalize**: Click "personalize content" button in chapters
3. **Translate**: Click "translate to Urdu" button in chapters

### Subagents
- **TranslationSubagent**: Converts content to Urdu
- **PersonalizationSubagent**: Adapts content to user background

## 🏗️ Architecture

The application consists of:
- **Frontend**: Docusaurus site with integrated chatbot, auth, and personalization
- **Backend**: FastAPI with RAG services, authentication, and vector database integration
- **Database**: PostgreSQL for user data, Qdrant for vector embeddings
- **AI Services**: OpenAI integration for intelligent responses

## 📊 Hackathon Requirements Fulfilled

✅ **Docusaurus Book**: Complete textbook with Physical AI curriculum
✅ **RAG Chatbot**: Integrated with selection-based queries
✅ **Better Auth**: Custom JWT-based authentication with background questions
✅ **Personalization**: Content personalization based on user profile
✅ **Translation**: Urdu translation with subagent architecture
✅ **Reusable Intelligence**: Subagent system for AI tasks
✅ **Vercel Deployment**: Configured for easy deployment

## 👩‍💻 Author

**Fatima Riaz** - [GitHub Profile](https://github.com/fatimaRiaz531)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## 📄 License

MIT License - see the [LICENSE](LICENSE) file for details.

## 🎯 Panaversity Initiative

This project is part of the Panaversity initiative to create AI-native educational content. Learn more at [panaversity.org](https://panaversity.org).