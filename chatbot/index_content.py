"""
Content Indexing Script for RAG Chatbot
Chunks textbook content and uploads to Qdrant vector database
"""

import os
import json
import re
from pathlib import Path
from typing import List, Dict
import logging
from dotenv import load_dotenv
import time

from qdrant_client import QdrantClient
from openai import OpenAI
from qdrant_client.http.models import VectorParams, Distance

# Setup
load_dotenv()
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize clients
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL", "http://localhost:6333"),
    api_key=os.getenv("QDRANT_API_KEY")
)
openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# ============================================================================
# CONFIGURATION
# ============================================================================

COLLECTION_NAME = "robotics-textbook"
VECTOR_SIZE = 1536  # OpenAI embedding size
CHUNK_SIZE = 500  # Words per chunk
CHUNK_OVERLAP = 50  # Words of overlap between chunks

MODULES = [
    {"id": 1, "name": "module-1-ros2", "title": "ROS 2 Fundamentals"},
    {"id": 2, "name": "module-2-gazebo-unity", "title": "Digital Twin (Gazebo & Unity)"},
    {"id": 3, "name": "module-3-isaac", "title": "NVIDIA Isaac Platform"},
    {"id": 4, "name": "module-4-vla", "title": "Vision-Language-Action"}
]

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def split_text_into_chunks(text: str, chunk_size: int = CHUNK_SIZE, overlap: int = CHUNK_OVERLAP) -> List[str]:
    """Split text into overlapping chunks"""
    words = text.split()
    chunks = []
    
    for i in range(0, len(words), chunk_size - overlap):
        chunk = " ".join(words[i:i + chunk_size])
        if len(chunk.split()) > 50:  # Minimum chunk size
            chunks.append(chunk)
    
    return chunks

def extract_sections(content: str) -> List[Dict]:
    """Extract markdown sections from content"""
    sections = []
    
    # Find all level-2 headers (##)
    pattern = r'^## (.+?)$'
    matches = list(re.finditer(pattern, content, re.MULTILINE))
    
    for i, match in enumerate(matches):
        section_title = match.group(1)
        section_start = match.end()
        section_end = matches[i + 1].start() if i + 1 < len(matches) else len(content)
        section_content = content[section_start:section_end].strip()
        
        # Remove code blocks for cleaner text
        section_text = re.sub(r'```[\s\S]*?```', '', section_content)
        section_text = re.sub(r'`[^`]+`', '', section_text)
        
        if section_text.strip():
            sections.append({
                "title": section_title,
                "content": section_text,
                "content_with_code": section_content
            })
    
    return sections

def get_embedding(text: str) -> List[float]:
    """Get embedding from OpenAI API with retry logic"""
    max_retries = 3
    for attempt in range(max_retries):
        try:
            response = openai_client.embeddings.create(
                input=text,
                model="text-embedding-3-small"
            )
            return response.data[0].embedding
        except Exception as e:
            if attempt < max_retries - 1:
                wait_time = 2 ** attempt
                logger.warning(f"Embedding failed (attempt {attempt + 1}), retrying in {wait_time}s: {e}")
                time.sleep(wait_time)
            else:
                logger.error(f"Failed to get embedding after {max_retries} attempts: {e}")
                raise

def create_collection(collection_name: str):
    """Create Qdrant collection if it doesn't exist"""
    try:
        # Check if collection exists
        qdrant_client.get_collection(collection_name)
        logger.info(f"Collection '{collection_name}' already exists")
    except:
        # Create collection with correct API
        logger.info(f"Creating collection '{collection_name}'...")
        try:
            qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(size=VECTOR_SIZE, distance=Distance.COSINE)
            )
            logger.info("Collection created successfully")
        except Exception as e:
            logger.warning(f"Could not create collection (may already exist): {e}")

def index_module_content(module: Dict) -> int:
    """Index a single module's content"""
    points_added = 0
    module_id = module["id"]
    module_name = module["name"]
    module_title = module["title"]
    
    # files live one level up in the repo under book/docs/
    file_path = os.path.join("..", "book", "docs", f"{module_name}.md")
    
    if not os.path.exists(file_path):
        logger.warning(f"File not found: {file_path}")
        return 0
    
    logger.info(f"\n📖 Processing Module {module_id}: {module_title}")
    logger.info(f"   File: {file_path}")
    
    # Read content
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Extract sections
    sections = extract_sections(content)
    logger.info(f"   Found {len(sections)} sections")
    
    for section in sections:
        section_title = section["title"]
        section_content = section["content"]
        
        # Split into chunks
        chunks = split_text_into_chunks(section_content)
        
        for chunk_idx, chunk in enumerate(chunks):
            try:
                # Get embedding
                embedding = get_embedding(chunk)
                
                # Create point ID (simple counter)
                point_id = hash(f"{module_id}-{section_title}-{chunk_idx}") % (10 ** 10)
                
                # Upsert to Qdrant
                qdrant_client.upsert(
                    collection_name=COLLECTION_NAME,
                    points=[
                        {
                            "id": abs(point_id),
                            "vector": embedding,
                            "payload": {
                                "module": f"Module {module_id}: {module_title}",
                                "section": section_title,
                                "content": chunk,
                                "module_id": module_id,
                                "chunk_index": chunk_idx,
                                "source_file": module_name
                            }
                        }
                    ]
                )
                
                points_added += 1
                
                if (points_added % 10) == 0:
                    logger.info(f"   ✅ Indexed {points_added} chunks...")
            
            except Exception as e:
                logger.error(f"Error indexing chunk {chunk_idx} of section '{section_title}': {e}")
                continue
    
    logger.info(f"   ✅ Module {module_id} complete: {points_added} chunks indexed")
    return points_added

# ============================================================================
# MAIN INDEXING FUNCTION
# ============================================================================

def index_all_content():
    """Index all module content to Qdrant"""
    logger.info("=" * 70)
    logger.info("🚀 ROBOTICS TEXTBOOK CONTENT INDEXING")
    logger.info("=" * 70)
    
    # Create collection
    create_collection(COLLECTION_NAME)
    
    total_points = 0
    
    # Index each module
    for module in MODULES:
        try:
            points = index_module_content(module)
            total_points += points
        except Exception as e:
            logger.error(f"Error processing module {module['id']}: {e}")
    
    logger.info("\n" + "=" * 70)
    logger.info(f"✅ INDEXING COMPLETE")
    logger.info(f"   Total chunks indexed: {total_points}")
    logger.info(f"   Collection: {COLLECTION_NAME}")
    logger.info("=" * 70)
    
    # Verify
    try:
        collection_info = qdrant_client.get_collection(COLLECTION_NAME)
        logger.info(f"\n📊 Collection Stats:")
        logger.info(f"   Vectors: {collection_info.points_count}")
        logger.info(f"   Vector size: {VECTOR_SIZE}")
        logger.info(f"   Distance metric: COSINE")
    except Exception as e:
        logger.error(f"Error fetching collection info: {e}")

# ============================================================================
# TEST FUNCTION
# ============================================================================

def test_search(query: str):
    """Test search functionality"""
    logger.info(f"\n🔍 Testing search with query: '{query}'")
    
    try:
        # Get embedding for query
        query_embedding = get_embedding(query)
        
        # Search
        results = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            limit=3,
            with_payload=True
        )
        
        logger.info(f"   Found {len(results)} results:")
        for i, result in enumerate(results, 1):
            logger.info(f"\n   Result {i} (score: {result.score:.3f})")
            logger.info(f"   Module: {result.payload['module']}")
            logger.info(f"   Section: {result.payload['section']}")
            logger.info(f"   Preview: {result.payload['content'][:100]}...")
    
    except Exception as e:
        logger.error(f"Search failed: {e}")

# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    import sys
    
    # Index all content
    index_all_content()
    
    # Test if query provided
    if len(sys.argv) > 1:
        test_query = " ".join(sys.argv[1:])
        test_search(test_query)
    else:
        # Default test
        test_search("How does ROS 2 work?")
        test_search("What is reinforcement learning in robotics?")
        test_search("How to simulate a robot in Gazebo?")
