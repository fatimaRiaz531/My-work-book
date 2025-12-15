from typing import List, Dict, Any
from openai import OpenAI
from uuid import uuid4
from ..config import settings


class EmbeddingGenerator:
    def __init__(self):
        self.openai_client = OpenAI(api_key=settings.openai_api_key)

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text using OpenAI API.

        Args:
            text: The text to generate embedding for

        Returns:
            List of floats representing the embedding vector
        """
        try:
            response = self.openai_client.embeddings.create(
                input=text,
                model=settings.embedding_model
            )
            return response.data[0].embedding
        except Exception as e:
            print(f"Error generating embedding for text: {str(e)}")
            raise

    def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts in a batch.

        Args:
            texts: List of texts to generate embeddings for

        Returns:
            List of embedding vectors
        """
        try:
            response = self.openai_client.embeddings.create(
                input=texts,
                model=settings.embedding_model
            )
            return [item.embedding for item in response.data]
        except Exception as e:
            print(f"Error generating batch embeddings: {str(e)}")
            raise

    def process_chunks(self, chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Process chunks to add embeddings and prepare for storage.

        Args:
            chunks: List of text chunks with metadata

        Returns:
            List of chunks with embeddings and unique IDs
        """
        processed_chunks = []

        # Process in batches to be more efficient with API calls
        batch_size = 10  # OpenAI recommends batches of 10-20
        for i in range(0, len(chunks), batch_size):
            batch = chunks[i:i + batch_size]
            texts = [chunk["content"] for chunk in batch]

            try:
                embeddings = self.generate_embeddings_batch(texts)

                for chunk, embedding in zip(batch, embeddings):
                    processed_chunk = {
                        "id": str(uuid4()),
                        "content": chunk["content"],
                        "embedding": embedding,
                        "metadata": {
                            "source_file": chunk["source_file"],
                            "section": chunk.get("section"),
                            "chapter": chunk.get("chapter")
                        }
                    }
                    processed_chunks.append(processed_chunk)

            except Exception as e:
                print(f"Error processing batch: {str(e)}")
                # Fallback: process one by one if batch failed
                for chunk in batch:
                    try:
                        embedding = self.generate_embedding(chunk["content"])
                        processed_chunk = {
                            "id": str(uuid4()),
                            "content": chunk["content"],
                            "embedding": embedding,
                            "metadata": {
                                "source_file": chunk["source_file"],
                                "section": chunk.get("section"),
                                "chapter": chunk.get("chapter")
                            }
                        }
                        processed_chunks.append(processed_chunk)
                    except Exception as e:
                        print(f"Skipping chunk due to error: {str(e)}")
                        continue

        return processed_chunks