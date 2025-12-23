from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List, Optional
from pydantic import BaseModel
from uuid import UUID, uuid4
import logging

from ..config.database import get_db, get_qdrant
from ..services.rag_service import RAGService
from ..services.mock_rag_service import MockRAGService
from ..services.retrieval_service import RetrievalService
from ..services.document_service import DocumentService

logger = logging.getLogger(__name__)

router = APIRouter()

# Pydantic models for request/response
class Message(BaseModel):
    role: str
    content: str


class ChatRequest(BaseModel):
    message: str
    session_id: Optional[str] = None
    history: Optional[List[Message]] = []


class SelectionChatRequest(BaseModel):
    message: str
    selection: str
    session_id: Optional[str] = None
    history: Optional[List[Message]] = []


class ChatResponse(BaseModel):
    response: str
    sources: List[dict]
    session_id: str
    timestamp: str


@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    request: ChatRequest,
    db: Session = Depends(get_db)
):
    """
    Chat endpoint for global book queries.
    Uses the entire book corpus to answer questions.
    """
    try:
        # Validation
        if not request.message or not request.message.strip():
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Message cannot be empty"
            )

        if len(request.message) > 10000:  # Limit message length
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Message too long. Maximum 10000 characters allowed."
            )

        # Initialize services with the actual Qdrant client
        try:
            qdrant_client = get_qdrant()
            retrieval_service = RetrievalService(qdrant_client=qdrant_client)
            rag_service = RAGService(db, retrieval_service)

            # Convert history to the format expected by RAGService
            conversation_history = [{"role": msg.role, "content": msg.content} for msg in request.history]

            # Generate response using full book context
            result = rag_service.query_full_book(request.message)
        except Exception as e:
            logger.warning(f"Real RAG service failed: {str(e)}, falling back to mock service")
            # Fall back to mock service without retrieval service to avoid Qdrant connection
            rag_service = MockRAGService(db)
            result = rag_service.query_full_book(request.message)

        # Generate or use session ID
        session_id = request.session_id or str(uuid4())

        # Create response
        from datetime import datetime
        response = ChatResponse(
            response=result["response"],
            sources=result["sources"],
            session_id=session_id,
            timestamp=datetime.utcnow().isoformat()
        )

        logger.info(f"Chat response generated successfully for session {session_id}")
        return response

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.post("/chat/selection", response_model=ChatResponse)
async def chat_selection_endpoint(
    request: SelectionChatRequest,
    db: Session = Depends(get_db)
):
    """
    Chat endpoint for selection-based queries.
    Answers questions based only on the user-provided text selection.
    """
    try:
        # Validation
        if not request.message or not request.message.strip():
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Message cannot be empty"
            )

        if not request.selection or not request.selection.strip():
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Selection cannot be empty"
            )

        if len(request.message) > 10000:  # Limit message length
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Message too long. Maximum 10000 characters allowed."
            )

        if len(request.selection) > 50000:  # Limit selection length
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Selection too long. Maximum 50000 characters allowed."
            )

        # Initialize services with the actual Qdrant client
        try:
            qdrant_client = get_qdrant()
            retrieval_service = RetrievalService(qdrant_client=qdrant_client)
            rag_service = RAGService(db, retrieval_service)

            # Convert history to the format expected by RAGService
            conversation_history = [{"role": msg.role, "content": msg.content} for msg in request.history]

            # Generate response using only the selected text as context
            result = rag_service.query_selection_only(request.message, request.selection)
        except Exception as e:
            logger.warning(f"Real RAG selection service failed: {str(e)}, falling back to mock service")
            # Fall back to mock service without retrieval service to avoid Qdrant connection
            rag_service = MockRAGService(db)
            result = rag_service.query_selection_only(request.message, request.selection)

        # Generate or use session ID
        session_id = request.session_id or str(uuid4())

        # Create response
        from datetime import datetime
        response = ChatResponse(
            response=result["response"],
            sources=result["sources"],
            session_id=session_id,
            timestamp=datetime.utcnow().isoformat()
        )

        logger.info(f"Selection chat response generated successfully for session {session_id}")
        return response

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Error in chat selection endpoint: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )