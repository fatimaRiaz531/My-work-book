from sqlalchemy.orm import Session
from typing import List, Optional
from ..models.document import Document
from uuid import UUID


class DocumentService:
    @staticmethod
    def create_document(
        db: Session,
        content: str,
        source_file: str,
        section: Optional[str] = None,
        chapter: Optional[str] = None,
        page_number: Optional[int] = None
    ) -> Document:
        """Create a new document entry in the database."""
        db_document = Document(
            content=content,
            source_file=source_file,
            section=section,
            chapter=chapter,
            page_number=page_number
        )
        db.add(db_document)
        db.commit()
        db.refresh(db_document)
        return db_document

    @staticmethod
    def get_document(db: Session, document_id: UUID) -> Optional[Document]:
        """Get a document by its ID."""
        return db.query(Document).filter(Document.id == document_id).first()

    @staticmethod
    def get_documents_by_source(db: Session, source_file: str) -> List[Document]:
        """Get all documents from a specific source file."""
        return db.query(Document).filter(Document.source_file == source_file).all()

    @staticmethod
    def get_all_documents(db: Session) -> List[Document]:
        """Get all documents."""
        return db.query(Document).all()