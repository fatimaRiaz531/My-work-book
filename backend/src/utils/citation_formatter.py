from typing import List, Dict, Any
from ..models.document import Document


def format_citations(sources: List[Dict[str, Any]]) -> str:
    """
    Format the citations from the retrieved sources.

    Args:
        sources: List of source documents with metadata

    Returns:
        Formatted citation string
    """
    if not sources:
        return ""

    citation_text = "\n\nSources cited:\n"
    for i, source in enumerate(sources, 1):
        source_info = []

        if source.get("source_file"):
            source_info.append(f"File: {source['source_file']}")

        if source.get("section"):
            source_info.append(f"Section: {source['section']}")

        if source.get("page_number") is not None:
            source_info.append(f"Page: {source['page_number']}")

        if source_info:
            citation_text += f"{i}. {' | '.join(source_info)}\n"

    return citation_text.rstrip()


def format_sources_for_response(sources: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    Format sources for API response with essential information only.

    Args:
        sources: List of source documents with metadata

    Returns:
        List of formatted source dictionaries
    """
    formatted_sources = []

    for source in sources:
        formatted_source = {
            "id": source.get("id"),
            "content": source.get("content", "")[:200] + "..." if len(source.get("content", "")) > 200 else source.get("content", ""),
            "source_file": source.get("source_file", ""),
            "section": source.get("section", ""),
            "page_number": source.get("page_number")
        }
        formatted_sources.append(formatted_source)

    return formatted_sources