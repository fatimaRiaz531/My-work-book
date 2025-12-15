from sqlalchemy import Column, Integer, String, DateTime, Text, UUID, JSON
from sqlalchemy.dialects.postgresql import UUID as PostgresUUID
from sqlalchemy.sql import func
import uuid
from ..config.database import Base


class Session(Base):
    __tablename__ = "sessions"

    id = Column(PostgresUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_token = Column(String, unique=True, index=True, nullable=False)  # Secure session identifier
    user_id = Column(String, nullable=True)  # Optional user identifier (anonymous if null)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())
    expires_at = Column(DateTime(timezone=True), nullable=False)  # When the session expires