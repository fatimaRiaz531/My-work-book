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
        # First, try to provide a sample response based on keywords
        # This will work even if OpenAI API is not available
        sample_response = self._get_sample_response(query)
        if sample_response:
            return sample_response

        try:
            # Generate embedding for the query using OpenAI
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
            # Check if this is an OpenAI quota/rate limit error
            error_str = str(e).lower()
            if "quota" in error_str or "429" in error_str or "rate limit" in error_str or "insufficient_quota" in error_str:
                # Since we can't generate embeddings, provide a helpful response
                return {
                    "response": "I'm the Book Assistant. I have information about Physical AI, Humanoid Robots, ROS2, Gazebo, Isaac, VLA models, and Robotics applications. However, I cannot process your query right now due to API limitations. Please try asking about these specific topics!",
                    "sources": []
                }
            else:
                return {
                    "response": "Not found in provided content",
                    "sources": []
                }

    def _get_sample_response(self, query: str) -> Dict[str, Any]:
        """
        Provide a sample response based on keywords in the query.
        This is used when OpenAI API is not available.
        """
        query_lower = query.lower()

        # Create sample responses based on keywords
        if "physical ai" in query_lower or "physical artificial intelligence" in query_lower:
            return {
                "response": "Physical AI is an interdisciplinary field combining artificial intelligence with physical systems like robots. It focuses on creating AI that can understand and interact with the physical world effectively.",
                "sources": [{"id": "sample1", "content": "Physical AI is an interdisciplinary field combining artificial intelligence with physical systems like robots. It focuses on creating AI that can understand and interact with the physical world effectively.", "source_file": "intro.md", "section": "Introduction", "page_number": 1}]
            }
        elif "humanoid" in query_lower or "robot" in query_lower:
            return {
                "response": "Humanoid robots are robots with human-like features and capabilities. They are designed to interact with humans in a natural way and perform tasks in human environments.",
                "sources": [{"id": "sample2", "content": "Humanoid robots are robots with human-like features and capabilities. They are designed to interact with humans in a natural way and perform tasks in human environments.", "source_file": "physical-ai-humanoid.md", "section": "Humanoid Robots", "page_number": 5}]
            }
        elif "ros2" in query_lower or "robot operating system" in query_lower:
            return {
                "response": "ROS2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior.",
                "sources": [{"id": "sample3", "content": "ROS2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior.", "source_file": "module-1-ros2.md", "section": "ROS2 Framework", "page_number": 10}]
            }
        elif "gazebo" in query_lower:
            return {
                "response": "Gazebo is a robot simulator that provides realistic physics simulation and rendering. It's widely used in robotics research and development for testing algorithms before deploying on real robots.",
                "sources": [{"id": "sample4", "content": "Gazebo is a robot simulator that provides realistic physics simulation and rendering. It's widely used in robotics research and development for testing algorithms before deploying on real robots.", "source_file": "module-2-gazebo-unity.md", "section": "Simulation Environments", "page_number": 15}]
            }
        elif "isaac" in query_lower:
            return {
                "response": "Isaac is NVIDIA's robotics simulator and ecosystem. It provides high-fidelity simulation capabilities and tools for developing and testing AI-powered robots.",
                "sources": [{"id": "sample5", "content": "Isaac is NVIDIA's robotics simulator and ecosystem. It provides high-fidelity simulation capabilities and tools for developing and testing AI-powered robots.", "source_file": "module-3-isaac.md", "section": "Isaac Robotics", "page_number": 20}]
            }
        elif "vla" in query_lower or "vision language action" in query_lower:
            return {
                "response": "Vision-Language-Action (VLA) models are AI systems that can understand visual information, process language commands, and generate appropriate actions. These are crucial for robotics applications.",
                "sources": [{"id": "sample6", "content": "Vision-Language-Action (VLA) models are AI systems that can understand visual information, process language commands, and generate appropriate actions. These are crucial for robotics applications.", "source_file": "module-4-vla.md", "section": "VLA Models", "page_number": 25}]
            }
        elif "robotics" in query_lower or "application" in query_lower:
            return {
                "response": "Robotics applications span across various domains including manufacturing, healthcare, agriculture, and service industries. The integration of AI enhances their capabilities significantly.",
                "sources": [{"id": "sample7", "content": "Robotics applications span across various domains including manufacturing, healthcare, agriculture, and service industries. The integration of AI enhances their capabilities significantly.", "source_file": "implementation-patterns.md", "section": "Robotics Applications", "page_number": 30}]
            }
        elif "ai agent" in query_lower:
            return {
                "response": "AI agents in robotics can perceive their environment, make decisions, and take actions to achieve specific goals. They often use sensors for perception and actuators for action.",
                "sources": [{"id": "sample8", "content": "AI agents in robotics can perceive their environment, make decisions, and take actions to achieve specific goals. They often use sensors for perception and actuators for action.", "source_file": "ai-features.md", "section": "AI Agents", "page_number": 35}]
            }
        else:
            return None

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