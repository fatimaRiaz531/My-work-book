from sqlalchemy import Column, Integer, String, DateTime, Text
from sqlalchemy.sql import func
import uuid
from ..config.database import Base


class Document(Base):
    __tablename__ = "documents"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    content = Column(Text, nullable=False)  # The actual content of the document chunk
    source_file = Column(String, nullable=False)  # Original file path from Docusaurus project
    section = Column(String, nullable=True)  # Section name or heading
    chapter = Column(String, nullable=True)  # Chapter name if applicable
    page_number = Column(Integer, nullable=True)  # Page number if available
    # embedding_vector is stored in Qdrant, not in Postgres
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())