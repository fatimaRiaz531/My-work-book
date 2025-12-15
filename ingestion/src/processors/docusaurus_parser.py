import os
import re
from pathlib import Path
from typing import List, Dict, Any
from ..config import settings


class DocusaurusParser:
    def __init__(self, source_directory: str = None):
        self.source_directory = source_directory or settings.source_directory

    def parse_all_files(self) -> List[Dict[str, Any]]:
        """
        Parse all supported files in the source directory.

        Returns:
            List of dictionaries containing file content and metadata
        """
        all_content = []

        source_path = Path(self.source_directory)
        if not source_path.exists():
            raise FileNotFoundError(f"Source directory does not exist: {self.source_directory}")

        # Find all supported files
        for file_path in source_path.rglob("*"):
            if file_path.suffix.lstrip(".") in settings.supported_formats:
                try:
                    content = self.parse_file(file_path)
                    if content:
                        all_content.append(content)
                except Exception as e:
                    print(f"Error parsing file {file_path}: {str(e)}")

        return all_content

    def parse_file(self, file_path: Path) -> Dict[str, Any]:
        """
        Parse a single file and extract content with metadata.

        Args:
            file_path: Path to the file to parse

        Returns:
            Dictionary containing content and metadata
        """
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract frontmatter if present (common in Docusaurus/MDX files)
        frontmatter = {}
        content_without_frontmatter = content

        # Look for YAML frontmatter
        frontmatter_match = re.match(r'^---\s*\n(.*?)\n---\s*\n(.*)', content, re.DOTALL)
        if frontmatter_match:
            frontmatter_text = frontmatter_match.group(1)
            content_without_frontmatter = frontmatter_match.group(2)

            # Simple frontmatter parsing (for now)
            for line in frontmatter_text.split('\n'):
                if ':' in line:
                    key, value = line.split(':', 1)
                    frontmatter[key.strip()] = value.strip().strip('"\'')

        # Extract potential section/chapter info from filename or content
        relative_path = str(file_path.relative_to(Path(self.source_directory)))
        section = self._extract_section_info(content_without_frontmatter, relative_path)
        chapter = self._extract_chapter_info(content_without_frontmatter, relative_path)

        return {
            "content": content_without_frontmatter,
            "source_file": relative_path,
            "section": section,
            "chapter": chapter,
            "page_number": None,  # Not applicable for web-based content
            "frontmatter": frontmatter
        }

    def _extract_section_info(self, content: str, file_path: str) -> str:
        """
        Extract section information from content or file path.

        Args:
            content: File content
            file_path: Relative file path

        Returns:
            Section name or identifier
        """
        # Look for H1 heading which often represents the main section
        h1_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
        if h1_match:
            return h1_match.group(1).strip()

        # Fallback to filename
        return Path(file_path).stem.replace('_', ' ').replace('-', ' ').title()

    def _extract_chapter_info(self, content: str, file_path: str) -> str:
        """
        Extract chapter information from content or file path.

        Args:
            content: File content
            file_path: Relative file path

        Returns:
            Chapter name or identifier
        """
        # For Docusaurus, chapters might be represented by directory structure
        path_parts = Path(file_path).parts
        if len(path_parts) > 1:
            # Use the parent directory name as the chapter
            return path_parts[-2].replace('_', ' ').replace('-', ' ').title()

        # Fallback to first part of filename if it looks like a chapter number
        filename = Path(file_path).stem
        if re.match(r'^\d+', filename):
            return filename.split('_', 1)[0] if '_' in filename else filename

        return "General"