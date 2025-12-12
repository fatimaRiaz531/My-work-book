# Physical AI & Humanoid Robotics Textbook

An AI-native technical textbook built with Docusaurus, featuring an integrated RAG chatbot, user authentication, content personalization, and Urdu translation.

## 🚀 Features

### ✅ Base Requirements (100 points)
- **AI-Native Spec-Driven Book**: Docusaurus site with 4 comprehensive modules
- **RAG Chatbot**: FastAPI backend with Qdrant vector database and OpenAI integration
- **GitHub Pages Deployment**: Automated CI/CD workflow

### ✅ Bonus Features (+200 points)
- **Better-Auth Integration**: Signup/Signin with user background questions
- **Content Personalization**: Per-chapter personalization based on user background
- **Urdu Translation**: One-click translation to Urdu for each chapter
- **Selection-based Q&A**: Answer questions from highlighted text

## 📁 Project Structure

```
humanoid-robotic-book/
├── book/                    # Docusaurus frontend
│   ├── docs/               # Textbook content (4 modules)
│   ├── src/
│   │   ├── components/     # React components (Chatbot, Auth, etc.)
│   │   └── theme/          # Docusaurus theme overrides
│   └── docusaurus.config.ts
├── chatbot/                 # FastAPI backend
│   ├── main.py             # Main API endpoints
│   ├── auth.py             # Authentication module
│   ├── index_content.py    # Content indexing script
│   └── requirements.txt
└── .github/workflows/       # GitHub Actions for deployment
```

## 🛠️ Setup Instructions

### Prerequisites
- Node.js >= 20.0
- Python 3.9+
- OpenAI API key
- Qdrant Cloud account (free tier available)

### Frontend Setup

```bash
cd book
npm install
npm start  # Runs on http://localhost:3000
```

### Backend Setup

```bash
cd chatbot
pip install -r requirements.txt

# Create .env file
cp .env.example .env
# Add your API keys:
# OPENAI_API_KEY=your_key_here
# QDRANT_URL=your_qdrant_url
# QDRANT_API_KEY=your_qdrant_key

# Index content
python index_content.py

# Run server
python -m uvicorn main:app --reload
```

## 📚 API Endpoints

### Chatbot
- `POST /query` - General RAG query
- `POST /query/selected` - Query with selected text

### Authentication
- `POST /auth/signup` - User signup with background questions
- `POST /auth/signin` - User signin
- `GET /auth/me` - Get current user profile

### Personalization & Translation
- `POST /personalize` - Personalize chapter content
- `POST /translate` - Translate content to Urdu

## 🎯 Hackathon Requirements Status

| Requirement | Status | Points |
|------------|--------|--------|
| AI-Native Book (Docusaurus) | ✅ Complete | 100 |
| RAG Chatbot (FastAPI + Qdrant) | ✅ Complete | +50 |
| Better-Auth Signup/Signin | ✅ Complete | +50 |
| Content Personalization | ✅ Complete | +50 |
| Urdu Translation | ✅ Complete | +50 |
| Claude Code Subagents | ⏳ Pending | +50 |

**Total: 300/350 points**

## 🌐 Deployment

### GitHub Pages
The site is automatically deployed via GitHub Actions when code is pushed to `main` branch.

**Live URL**: https://fatimaRiaz531.github.io/My-work-book/

### Backend Deployment
Deploy the FastAPI backend to:
- Vercel (recommended)
- Railway
- Render
- Any Python hosting service

Set environment variables:
- `OPENAI_API_KEY`
- `QDRANT_URL`
- `QDRANT_API_KEY`
- `SECRET_KEY` (for JWT tokens)
- `CORS_ORIGINS` (your frontend URL)

## 📖 Usage

### For Readers
1. Visit the deployed site
2. Sign up with your background information
3. Navigate to any chapter
4. Use "Personalize Content" to adjust difficulty
5. Use "Translate to Urdu" for Urdu version
6. Click "Ask the Book" to chat with the RAG chatbot

### For Developers
- See `chatbot/README.md` for backend documentation
- See `book/README.md` for frontend documentation
- Check `.github/workflows/deploy.yml` for CI/CD setup

## 🔧 Technology Stack

**Frontend:**
- Docusaurus 3.9.2
- React 19
- TypeScript

**Backend:**
- FastAPI 0.104.1
- Qdrant Client 1.16.1
- OpenAI API
- Python-JOSE (JWT)
- Passlib (password hashing)

## 📝 License

This project is part of the Panaversity Hackathon submission.

## 👥 Contributors

- Fatima Riaz (fatimaRiaz531)

---

**Repository**: https://github.com/fatimaRiaz531/My-work-book

