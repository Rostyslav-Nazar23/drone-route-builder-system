"""Database connection and session management."""
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, Session
from sqlalchemy.ext.declarative import declarative_base
from typing import Optional
import os

Base = declarative_base()

# Database URL from environment or default
DATABASE_URL = os.getenv(
    "DATABASE_URL",
    "postgresql://postgres:postgres@localhost:5432/drone_routes"
)

engine = None
SessionLocal = None


def init_db(database_url: Optional[str] = None):
    """Initialize database connection.
    
    Args:
        database_url: PostgreSQL connection string (optional)
    """
    global engine, SessionLocal
    
    url = database_url or DATABASE_URL
    engine = create_engine(url, echo=False)
    SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
    
    # Create tables
    Base.metadata.create_all(bind=engine)


def get_db() -> Session:
    """Get database session.
    
    Returns:
        Database session
    """
    if SessionLocal is None:
        init_db()
    
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


def close_db():
    """Close database connection."""
    global engine
    if engine:
        engine.dispose()

