from sqlalchemy import Column, Integer, String, DateTime, Text, UUID, JSON
from sqlalchemy.dialects.postgresql import UUID as PostgresUUID
from sqlalchemy.sql import func
import uuid
from ..config.database import Base


class Response(Base):
    __tablename__ = "responses"

    id = Column(PostgresUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(PostgresUUID(as_uuid=True), nullable=False)  # Reference to the parent session
    role = Column(String, nullable=False)  # Either "user" or "assistant"
    content = Column(Text, nullable=False)  # The message content
    sources = Column(JSON, nullable=True)  # List of source documents referenced in the response
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    sequence_number = Column(Integer, nullable=False)  # Order of the message in the conversation