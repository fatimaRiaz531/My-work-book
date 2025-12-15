from typing import List, Dict, Any, Optional
from openai import OpenAI
from sqlalchemy.orm import Session
from ..config.settings import settings
from ..services.retrieval_service import RetrievalService
from ..services.document_service import DocumentService
import logging
import json

logger = logging.getLogger(__name__)


class RAGService:
    def __init__(self, db: Session, retrieval_service: RetrievalService):
        self.db = db
        self.retrieval_service = retrieval_service
        self.openai_client = OpenAI(api_key=settings.openai_api_key)

    def generate_response(
        self,
        query: str,
        context_documents: List[Dict[str, Any]],
        conversation_history: Optional[List[Dict[str, str]]] = None
    ) -> Dict[str, Any]:
        """
        Generate a response using the RAG approach with OpenAI.

        Args:
            query: The user's query
            context_documents: Retrieved documents to use as context
            conversation_history: Previous conversation history (optional)

        Returns:
            Dictionary containing the response and sources
        """
        try:
            # Prepare context from retrieved documents
            context_text = "\n\n".join([doc["content"] for doc in context_documents])

            # Prepare the prompt for the OpenAI agent
            if context_text.strip():
                system_prompt = f"""
                You are an AI assistant that answers questions based on the provided context from a technical book.
                Only use information from the context provided below to answer the user's question.
                If the answer is not found in the provided context, respond with "Not found in provided content".
                Always cite the source of information by referencing the source file, section, and page number when available.

                Context:
                {context_text}
                """
            else:
                system_prompt = "You are an AI assistant. If the user asks a question, respond with 'Not found in provided content' since no relevant context was found."

            # Prepare messages for the conversation
            messages = [
                {"role": "system", "content": system_prompt}
            ]

            # Add conversation history if provided
            if conversation_history:
                for message in conversation_history:
                    messages.append({
                        "role": message.get("role", "user"),
                        "content": message.get("content", "")
                    })

            # Add the current user query
            messages.append({"role": "user", "content": query})

            # Call the OpenAI API to generate the response
            response = self.openai_client.chat.completions.create(
                model=settings.openai_model,
                messages=messages,
                temperature=0.3,  # Lower temperature for more consistent, factual responses
                max_tokens=1000
            )

            # Extract the generated text
            generated_text = response.choices[0].message.content

            # Prepare the response with sources
            sources = []
            for doc in context_documents:
                source_info = {
                    "id": doc.get("id"),
                    "content": doc.get("content", "")[:200] + "..." if len(doc.get("content", "")) > 200 else doc.get("content", ""),
                    "source_file": doc.get("source_file", ""),
                    "section": doc.get("section", ""),
                    "page_number": doc.get("page_number")
                }
                sources.append(source_info)

            return {
                "response": generated_text,
                "sources": sources
            }

        except Exception as e:
            logger.error(f"Error generating response: {str(e)}")
            return {
                "response": "Not found in provided content",
                "sources": []
            }

    def query_full_book(self, query: str, top_k: int = 5) -> Dict[str, Any]:
        """
        Query the entire book corpus for relevant information.

        Args:
            query: The user's query
            top_k: Number of top results to retrieve

        Returns:
            Dictionary containing the response and sources
        """
        try:
            # First, generate embedding for the query using OpenAI
            response = self.openai_client.embeddings.create(
                input=query,
                model="text-embedding-ada-002"
            )
            query_embedding = response.data[0].embedding

            # Search for similar documents in Qdrant
            context_documents = self.retrieval_service.search_similar(
                query_embedding=query_embedding,
                collection_name=settings.qdrant_collection_name,
                top_k=top_k
            )

            # Generate response using the context
            return self.generate_response(query, context_documents)

        except Exception as e:
            logger.error(f"Error in full book query: {str(e)}")
            return {
                "response": "Not found in provided content",
                "sources": []
            }

    def query_selection_only(self, query: str, selected_text: str) -> Dict[str, Any]:
        """
        Query only the provided selected text for relevant information.

        Args:
            query: The user's query about the selected text
            selected_text: The text that was selected by the user

        Returns:
            Dictionary containing the response and sources
        """
        try:
            # Create a context document from the selected text
            context_documents = [{
                "id": "selected_text",
                "content": selected_text,
                "source_file": "user_selection",
                "section": "user_selected",
                "page_number": None,
                "score": 1.0  # Perfect match score
            }]

            # Generate response using only the selected text as context
            return self.generate_response(query, context_documents)

        except Exception as e:
            logger.error(f"Error in selection-only query: {str(e)}")
            return {
                "response": "Not found in provided content",
                "sources": []
            }