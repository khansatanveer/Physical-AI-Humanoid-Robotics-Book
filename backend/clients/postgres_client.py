from sqlalchemy import create_engine, Column, String, Integer, DateTime, Text, Float, Boolean
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.sql import func
from typing import Optional, List
import uuid
from datetime import datetime

from config import settings


# SQLAlchemy setup
engine = create_engine(
    settings.neon_database_url,
    pool_size=10,
    max_overflow=20,
    pool_pre_ping=True,  # Verify connections before use
    pool_recycle=300,    # Recycle connections every 5 minutes
)

SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()


class ChatInteraction(Base):
    """Database model for chat interactions."""
    __tablename__ = "chat_interactions"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column(String, nullable=True)  # Optional user identifier
    session_id = Column(String, nullable=True)  # Session identifier for conversation history
    query = Column(Text, nullable=False)  # The user's original question
    response = Column(Text, nullable=False)  # The system's generated response
    query_type = Column(String, nullable=False)  # "global" or "selection"
    selected_text = Column(Text, nullable=True)  # Text selected by user (for selection queries only)
    retrieved_chunks = Column(String, nullable=True)  # JSON string of retrieved chunk IDs
    confidence_score = Column(Float, nullable=True)  # Confidence level in the response (0-1)
    source_attribution = Column(Text, nullable=True)  # JSON string of source references
    timestamp = Column(DateTime, default=func.now(), nullable=False)


class QueryLog(Base):
    """Database model for query logs."""
    __tablename__ = "query_logs"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    query = Column(Text, nullable=False)  # The original query
    response_status = Column(String, nullable=False)  # "success", "error", "no_content"
    retrieval_time = Column(Float, nullable=True)  # Time taken for content retrieval (ms)
    generation_time = Column(Float, nullable=True)  # Time taken for response generation (ms)
    total_time = Column(Float, nullable=True)  # Total processing time (ms)
    retrieved_count = Column(Integer, nullable=True)  # Number of content chunks retrieved
    user_agent = Column(String, nullable=True)  # Client information (optional)
    ip_address = Column(String, nullable=True)  # User IP for analytics (respecting privacy)
    timestamp = Column(DateTime, default=func.now(), nullable=False)


class UserFeedback(Base):
    """Database model for user feedback."""
    __tablename__ = "user_feedback"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    interaction_id = Column(String, nullable=False)  # Reference to chat interaction
    rating = Column(Integer, nullable=True)  # Numerical rating (1-5 scale)
    thumbs = Column(String, nullable=True)  # "up", "down", or null
    comment = Column(Text, nullable=True)  # Optional textual feedback
    useful = Column(Boolean, nullable=True)  # Whether the response was helpful
    accuracy_rating = Column(Integer, nullable=True)  # Accuracy rating (1-5 scale)
    timestamp = Column(DateTime, default=func.now(), nullable=False)


class BookContent(Base):
    """Database model for book content metadata (not the embeddings, which are in Qdrant)."""
    __tablename__ = "book_content_metadata"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    content_id = Column(String, nullable=False)  # ID from Qdrant
    module = Column(String, nullable=False)
    chapter = Column(String, nullable=False)
    section = Column(String, nullable=False)
    heading_path = Column(String, nullable=False)
    source_url = Column(String, nullable=False)
    content_preview = Column(Text, nullable=True)  # Short preview of the content
    created_at = Column(DateTime, default=func.now(), nullable=False)
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now(), nullable=False)


# Create tables
def init_db():
    """Initialize the database tables."""
    Base.metadata.create_all(bind=engine)


def get_db():
    """Dependency function to get database session."""
    db = SessionLocal()
    try:
        return db
    finally:
        pass  # Session will be closed by the caller


# Service class for database operations
class PostgresService:
    """Service class to handle Postgres database operations."""

    def __init__(self):
        """Initialize the Postgres service."""
        self.engine = engine
        self.SessionLocal = SessionLocal

    def save_chat_interaction(
        self,
        user_id: Optional[str],
        session_id: Optional[str],
        query: str,
        response: str,
        query_type: str,
        selected_text: Optional[str] = None,
        retrieved_chunks: Optional[str] = None,
        confidence_score: Optional[float] = None,
        source_attribution: Optional[str] = None
    ) -> str:
        """Save a chat interaction to the database."""
        db = self.SessionLocal()
        try:
            interaction = ChatInteraction(
                user_id=user_id,
                session_id=session_id,
                query=query,
                response=response,
                query_type=query_type,
                selected_text=selected_text,
                retrieved_chunks=retrieved_chunks,
                confidence_score=confidence_score,
                source_attribution=source_attribution
            )
            db.add(interaction)
            db.commit()
            db.refresh(interaction)
            return interaction.id
        finally:
            db.close()

    def save_query_log(
        self,
        query: str,
        response_status: str,
        retrieval_time: Optional[float] = None,
        generation_time: Optional[float] = None,
        total_time: Optional[float] = None,
        retrieved_count: Optional[int] = None,
        user_agent: Optional[str] = None,
        ip_address: Optional[str] = None
    ) -> str:
        """Save a query log to the database."""
        db = self.SessionLocal()
        try:
            log = QueryLog(
                query=query,
                response_status=response_status,
                retrieval_time=retrieval_time,
                generation_time=generation_time,
                total_time=total_time,
                retrieved_count=retrieved_count,
                user_agent=user_agent,
                ip_address=ip_address
            )
            db.add(log)
            db.commit()
            db.refresh(log)
            return log.id
        finally:
            db.close()

    def save_user_feedback(
        self,
        interaction_id: str,
        rating: Optional[int] = None,
        thumbs: Optional[str] = None,
        comment: Optional[str] = None,
        useful: Optional[bool] = None,
        accuracy_rating: Optional[int] = None
    ) -> str:
        """Save user feedback to the database."""
        db = self.SessionLocal()
        try:
            feedback = UserFeedback(
                interaction_id=interaction_id,
                rating=rating,
                thumbs=thumbs,
                comment=comment,
                useful=useful,
                accuracy_rating=accuracy_rating
            )
            db.add(feedback)
            db.commit()
            db.refresh(feedback)
            return feedback.id
        finally:
            db.close()

    def get_chat_interactions_by_session(self, session_id: str) -> List[ChatInteraction]:
        """Get all chat interactions for a specific session."""
        db = self.SessionLocal()
        try:
            interactions = db.query(ChatInteraction).filter(
                ChatInteraction.session_id == session_id
            ).order_by(ChatInteraction.timestamp).all()
            return interactions
        finally:
            db.close()

    def get_recent_query_logs(self, limit: int = 100) -> List[QueryLog]:
        """Get recent query logs."""
        db = self.SessionLocal()
        try:
            logs = db.query(QueryLog).order_by(
                QueryLog.timestamp.desc()
            ).limit(limit).all()
            return logs
        finally:
            db.close()

    def health_check(self) -> bool:
        """Check if Postgres is accessible."""
        db = self.SessionLocal()
        try:
            # Try to execute a simple query
            db.execute("SELECT 1")
            return True
        except Exception:
            return False
        finally:
            db.close()


# Global instance
postgres_service = PostgresService()
init_db()