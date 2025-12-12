"""
RAG Chatbot Backend for Physical AI & Humanoid Robotics Textbook
Integrates FastAPI, Qdrant vector database, and OpenAI API for semantic search
"""

from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import os
from dotenv import load_dotenv
import logging
from datetime import datetime, timedelta
from auth import (
    UserSignup, UserSignin, UserProfile, TokenResponse,
    create_access_token, get_current_user, create_user,
    get_user_by_email, verify_password
)

# Vector database and LLM imports
from qdrant_client import QdrantClient
from openai import OpenAI

# Load environment variables
load_dotenv()

# Initialize FastAPI app
app = FastAPI(
    title="Robotics Textbook RAG Chatbot",
    description="Question-answering system for Physical AI & Humanoid Robotics",
    version="1.0.0",
)

# Config
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION", "robotics-textbook")
EMBEDDING_MODEL = os.getenv("EMBEDDING_MODEL", "text-embedding-3-small")
CHAT_MODEL = os.getenv("CHAT_MODEL", "gpt-4-turbo-preview")

# CORS Configuration - Fix: Cannot use wildcard "*" with allow_credentials=True
# Default to specific localhost origins for development
default_cors_origins = "http://localhost:3000,http://localhost:3001"
cors_origins_env = os.getenv("CORS_ORIGINS", default_cors_origins)
cors_origins_list = [origin.strip() for origin in cors_origins_env.split(",")]

# Only enable credentials if not using wildcard "*"
# Browsers reject requests when credentials=true and origin="*"
has_wildcard = "*" in cors_origins_list
allow_creds = not has_wildcard

# Enable CORS for Docusaurus integration
app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins_list if not has_wildcard else ["*"],
    allow_credentials=allow_creds,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize clients
try:
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL", "http://localhost:6333"),
        api_key=os.getenv("QDRANT_API_KEY")
    )
    openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
except Exception as e:
    print(f"Warning: Could not initialize clients: {e}")
    qdrant_client = None
    openai_client = None

# Logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ============================================================================
# DATA MODELS
# ============================================================================

class QueryRequest(BaseModel):
    """User query request"""
    question: str
    top_k: Optional[int] = 7
    threshold: Optional[float] = 0.5

class Source(BaseModel):
    """Source reference"""
    module: str
    section: str
    content_preview: str
    relevance_score: float

class ChatResponse(BaseModel):
    """Chatbot response"""
    answer: str
    sources: List[Source]
    query_tokens: int
    response_tokens: int
    timestamp: str


class SelectionQuery(BaseModel):
    """User query with manually selected context"""
    question: str
    selected_text: str
    fallback_to_rag: bool = False

class HealthCheck(BaseModel):
    """Health check response"""
    status: str
    qdrant_connected: bool
    openai_connected: bool

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def get_embedding(text: str) -> List[float]:
    """Convert text to embedding using OpenAI API"""
    if not openai_client:
        raise HTTPException(status_code=503, detail="OpenAI client not initialized")
    
    try:
        response = openai_client.embeddings.create(
            input=text,
            model=EMBEDDING_MODEL,
        )
        return response.data[0].embedding
    except Exception as e:
        logger.error(f"Embedding error: {e}")
        raise HTTPException(status_code=500, detail=f"Embedding failed: {str(e)}")

def search_qdrant(query_embedding: List[float], top_k: int = 5) -> List[dict]:
    """Search Qdrant vector database for similar content"""
    if not qdrant_client:
        raise HTTPException(status_code=503, detail="Qdrant client not initialized")
    
    try:
        search_results = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True,
        )

        results = []
        for point in search_results:
            results.append(
                {
                    "module": point.payload.get("module", "Unknown"),
                    "section": point.payload.get("section", "Unknown"),
                    "content": point.payload.get("content", ""),
                    "score": point.score,
                }
            )
        return results
    except Exception as e:
        logger.error(f"Search error: {e}")
        raise HTTPException(status_code=500, detail=f"Search failed: {str(e)}")

def generate_answer(question: str, context: str) -> tuple[str, int, int]:
    """Generate answer using GPT-4 with context"""
    if not openai_client:
        raise HTTPException(status_code=503, detail="OpenAI client not initialized")
    
    try:
        system_prompt = """You are an expert in Physical AI and Humanoid Robotics. 
        Answer questions based on the provided context from the textbook.
        Always cite your sources using [Source: Module X - Section Name].
        If the context doesn't contain relevant information, say so clearly.
        Keep answers concise but thorough."""
        
        user_message = f"""Based on this context from the textbook:

{context}

Please answer this question:
{question}

Remember to cite the sources where you found the information."""
        
        response = openai_client.chat.completions.create(
            model=CHAT_MODEL,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_message}
            ],
            temperature=0.3,
            max_tokens=1024
        )
        
        answer = response.choices[0].message.content
        query_tokens = response.usage.prompt_tokens
        response_tokens = response.usage.completion_tokens
        
        return answer, query_tokens, response_tokens
    except Exception as e:
        logger.error(f"Generation error: {e}")
        raise HTTPException(status_code=500, detail=f"Answer generation failed: {str(e)}")

# ============================================================================
# API ENDPOINTS
# ============================================================================

@app.get("/health", response_model=HealthCheck)
async def health_check():
    """Check API and service health"""
    qdrant_ok = False
    openai_ok = False
    
    try:
        if qdrant_client:
            qdrant_client.get_collections()
            qdrant_ok = True
    except Exception as e:
        logger.warning(f"Qdrant check failed: {e}")
    
    try:
        if openai_client:
            openai_client.models.list()
            openai_ok = True
    except Exception as e:
        logger.warning(f"OpenAI check failed: {e}")
    
    return HealthCheck(
        status="healthy" if (qdrant_ok and openai_ok) else "degraded",
        qdrant_connected=qdrant_ok,
        openai_connected=openai_ok
    )

@app.post("/query", response_model=ChatResponse)
async def query_chatbot(request: QueryRequest):
    """
    Main chatbot endpoint
    1. Convert question to embedding
    2. Search Qdrant for relevant content
    3. Generate answer with GPT-4
    4. Return response with citations
    """
    
    logger.info(f"Query received: {request.question[:100]}...")
    
    # Step 1: Get question embedding
    question_embedding = get_embedding(request.question)
    
    # Step 2: Search Qdrant
    search_results = search_qdrant(question_embedding, top_k=request.top_k)
    
    if not search_results:
        return ChatResponse(
            answer="I couldn't find relevant information in the textbook to answer your question.",
            sources=[],
            query_tokens=0,
            response_tokens=0,
            timestamp=datetime.now().isoformat()
        )
    
    # Step 3: Build context from search results
    context_parts = []
    for result in search_results:
        context_parts.append(
            f"[{result['module']} - {result['section']}] {result['content'][:500]}..."
        )
    context = "\n\n".join(context_parts)
    
    # Step 4: Generate answer
    answer, query_tokens, response_tokens = generate_answer(request.question, context)
    
    # Step 5: Format sources
    sources = [
        Source(
            module=result["module"],
            section=result["section"],
            content_preview=result["content"][:200],
            relevance_score=round(result["score"], 3)
        )
        for result in search_results
    ]
    
    logger.info(f"Response generated: {len(answer)} chars, {len(sources)} sources")
    
    return ChatResponse(
        answer=answer,
        sources=sources,
        query_tokens=query_tokens,
        response_tokens=response_tokens,
        timestamp=datetime.now().isoformat()
    )


@app.post("/query/selected", response_model=ChatResponse)
async def query_with_selection(request: SelectionQuery):
    """
    Answer using only user-selected text; optional fallback to RAG.
    """
    logger.info(f"Selection query received: {request.question[:100]}...")

    if not request.selected_text.strip():
        if request.fallback_to_rag:
            return await query_chatbot(
                QueryRequest(question=request.question, top_k=7, threshold=0.5)
            )
        return ChatResponse(
            answer="Please highlight text to answer from, or enable fallback_to_rag.",
            sources=[],
            query_tokens=0,
            response_tokens=0,
            timestamp=datetime.now().isoformat(),
        )

    answer, query_tokens, response_tokens = generate_answer(
        request.question, request.selected_text
    )

    # Provide a synthetic source referencing the selection
    sources = [
        Source(
            module="User Selection",
            section="Highlighted Text",
            content_preview=request.selected_text[:200],
            relevance_score=1.0,
        )
    ]

    return ChatResponse(
        answer=answer,
        sources=sources,
        query_tokens=query_tokens,
        response_tokens=response_tokens,
        timestamp=datetime.now().isoformat(),
    )

@app.post("/auth/signup", response_model=TokenResponse)
async def signup(user_data: UserSignup):
    """User signup with background questions"""
    try:
        user = create_user(user_data)
        access_token_expires = timedelta(minutes=30 * 24 * 60)  # 30 days
        access_token = create_access_token(
            data={"sub": user["email"]}, expires_delta=access_token_expires
        )
        
        return TokenResponse(
            access_token=access_token,
            token_type="bearer",
            user=UserProfile(
                id=user["id"],
                email=user["email"],
                name=user["name"],
                software_background=user["software_background"],
                hardware_background=user["hardware_background"],
                robotics_experience=user["robotics_experience"],
                programming_languages=user.get("programming_languages"),
                preferred_learning_style=user.get("preferred_learning_style"),
                created_at=user["created_at"]
            )
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Signup error: {e}")
        raise HTTPException(status_code=500, detail="Signup failed")

@app.post("/auth/signin", response_model=TokenResponse)
async def signin(credentials: UserSignin):
    """User signin"""
    user = get_user_by_email(credentials.email)
    if not user or not verify_password(credentials.password, user["hashed_password"]):
        raise HTTPException(status_code=401, detail="Incorrect email or password")
    
    access_token_expires = timedelta(minutes=30 * 24 * 60)  # 30 days
    access_token = create_access_token(
        data={"sub": user["email"]}, expires_delta=access_token_expires
    )
    
    return TokenResponse(
        access_token=access_token,
        token_type="bearer",
        user=UserProfile(
            id=user["id"],
            email=user["email"],
            name=user["name"],
            software_background=user["software_background"],
            hardware_background=user["hardware_background"],
            robotics_experience=user["robotics_experience"],
            programming_languages=user.get("programming_languages"),
            preferred_learning_style=user.get("preferred_learning_style"),
            created_at=user["created_at"]
        )
    )

@app.get("/auth/me", response_model=UserProfile)
async def get_current_user_profile(current_user: dict = Depends(get_current_user)):
    """Get current user profile"""
    return UserProfile(
        id=current_user["id"],
        email=current_user["email"],
        name=current_user["name"],
        software_background=current_user["software_background"],
        hardware_background=current_user["hardware_background"],
        robotics_experience=current_user["robotics_experience"],
        programming_languages=current_user.get("programming_languages"),
        preferred_learning_style=current_user.get("preferred_learning_style"),
        created_at=current_user["created_at"]
    )

class PersonalizeRequest(BaseModel):
    """Personalization request"""
    chapter_id: str
    content: str
    user_background: dict

class TranslateRequest(BaseModel):
    """Translation request"""
    text: str
    target_language: str = "ur"

@app.post("/personalize")
async def personalize_content(request: PersonalizeRequest, current_user: dict = Depends(get_current_user)):
    """Personalize chapter content based on user background"""
    try:
        background_info = f"""
        User Background:
        - Software: {request.user_background.get('software', 'Unknown')}
        - Hardware: {request.user_background.get('hardware', 'Unknown')}
        - Robotics: {request.user_background.get('robotics', 'Unknown')}
        """
        
        prompt = f"""You are an expert educator. Personalize the following textbook chapter content 
        to match the user's background level. Adjust explanations, examples, and technical depth accordingly.
        
        {background_info}
        
        Original Content:
        {request.content}
        
        Provide the personalized version maintaining the same structure and markdown format."""
        
        response = openai_client.chat.completions.create(
            model=CHAT_MODEL,
            messages=[
                {"role": "system", "content": "You are an expert educational content personalizer."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.7,
            max_tokens=4000
        )
        
        personalized_content = response.choices[0].message.content
        
        return {
            "personalized_content": personalized_content,
            "chapter_id": request.chapter_id
        }
    except Exception as e:
        logger.error(f"Personalization error: {e}")
        raise HTTPException(status_code=500, detail=f"Personalization failed: {str(e)}")

@app.post("/translate")
async def translate_content(request: TranslateRequest):
    """Translate content to target language (Urdu)"""
    try:
        prompt = f"""Translate the following technical textbook content to {request.target_language} (Urdu).
        Maintain technical terms in English where appropriate, use Urdu script for explanations.
        Preserve markdown formatting.
        
        Content:
        {request.text}"""
        
        response = openai_client.chat.completions.create(
            model=CHAT_MODEL,
            messages=[
                {"role": "system", "content": "You are an expert translator specializing in technical content."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.3,
            max_tokens=4000
        )
        
        translated_text = response.choices[0].message.content
        
        return {
            "translated_text": translated_text,
            "target_language": request.target_language
        }
    except Exception as e:
        logger.error(f"Translation error: {e}")
        raise HTTPException(status_code=500, detail=f"Translation failed: {str(e)}")

@app.get("/")
async def root():
    """API root endpoint"""
    return {
        "name": "Robotics Textbook RAG Chatbot",
        "version": "1.0.0",
        "endpoints": {
            "health": "/health",
            "query": "/query",
            "auth": "/auth/signup, /auth/signin, /auth/me",
            "personalize": "/personalize",
            "translate": "/translate",
            "docs": "/docs"
        }
    }

# ============================================================================
# ERROR HANDLERS
# ============================================================================

@app.exception_handler(HTTPException)
async def http_exception_handler(request, exc):
    return {
        "error": exc.detail,
        "status_code": exc.status_code,
        "timestamp": datetime.now().isoformat()
    }

# ============================================================================
# RUN
# ============================================================================

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8000,
        log_level="info"
    )
