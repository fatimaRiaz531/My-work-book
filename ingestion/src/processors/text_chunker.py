import re
from typing import List, Dict, Any
from ..config import settings


class TextChunker:
    def __init__(self, chunk_size: int = None, chunk_overlap: int = None):
        self.chunk_size = chunk_size or settings.chunk_size
        self.chunk_overlap = chunk_overlap or settings.chunk_overlap

    def chunk_text(self, text: str, source_file: str, section: str = None, chapter: str = None) -> List[Dict[str, Any]]:
        """
        Split text into overlapping chunks.

        Args:
            text: The text to chunk
            source_file: Source file identifier
            section: Section name (optional)
            chapter: Chapter name (optional)

        Returns:
            List of dictionaries containing chunk content and metadata
        """
        if len(text) <= self.chunk_size:
            # If text is smaller than chunk size, return as single chunk
            return [{
                "content": text,
                "source_file": source_file,
                "section": section,
                "chapter": chapter
            }]

        chunks = []
        start_idx = 0

        while start_idx < len(text):
            # Determine the end index for this chunk
            end_idx = start_idx + self.chunk_size

            # If we're near the end, make sure to include the remainder
            if end_idx >= len(text):
                end_idx = len(text)
            else:
                # Try to break at sentence or paragraph boundary to avoid cutting sentences
                chunk_text = text[start_idx:end_idx]

                # Look for a good breaking point (sentence end, paragraph end, or whitespace)
                break_points = [
                    chunk_text.rfind('. ', 0, self.chunk_size),
                    chunk_text.rfind('! ', 0, self.chunk_size),
                    chunk_text.rfind('? ', 0, self.chunk_size),
                    chunk_text.rfind('\n\n', 0, self.chunk_size),
                    chunk_text.rfind('\n', 0, self.chunk_size),
                    chunk_text.rfind(' ', 0, self.chunk_size)
                ]

                # Find the best breaking point within the chunk
                best_break = max(bp for bp in break_points if bp != -1)

                if best_break > len(chunk_text) // 2:  # Only break if it's not cutting too early
                    end_idx = start_idx + best_break + 1  # +1 to include the breaking character

            # Extract the chunk
            chunk_content = text[start_idx:end_idx]

            chunks.append({
                "content": chunk_content,
                "source_file": source_file,
                "section": section,
                "chapter": chapter
            })

            # Move start index forward, considering overlap
            start_idx = end_idx - self.chunk_overlap

            # Ensure we don't get stuck in an infinite loop
            if start_idx <= start_idx:  # Previous start_idx
                start_idx += self.chunk_size

        return chunks

    def chunk_documents(self, documents: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Chunk multiple documents.

        Args:
            documents: List of documents with content and metadata

        Returns:
            List of chunked documents
        """
        all_chunks = []
        for doc in documents:
            chunks = self.chunk_text(
                text=doc["content"],
                source_file=doc["source_file"],
                section=doc.get("section"),
                chapter=doc.get("chapter")
            )
            all_chunks.extend(chunks)

        return all_chunks