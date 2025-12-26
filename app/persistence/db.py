"""Database connection and session management."""
from sqlalchemy import create_engine, inspect, text
from sqlalchemy.orm import sessionmaker, Session
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.exc import ProgrammingError, OperationalError
from typing import Optional
import os
from pathlib import Path
import logging

logger = logging.getLogger(__name__)

# Load .env file if it exists
try:
    from dotenv import load_dotenv
    env_path = Path(__file__).parent.parent.parent / '.env'
    if env_path.exists():
        load_dotenv(dotenv_path=env_path)
except ImportError:
    # python-dotenv not installed, skip
    pass

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
    
    # Clean up any conflicting types that might interfere with table creation
    # This can happen if a composite type or enum was previously created with the same name
    try:
        with engine.begin() as conn:  # Use begin() for transaction management
            # Try to drop any existing type named 'missions' (CASCADE handles dependencies)
            conn.execute(text("DROP TYPE IF EXISTS missions CASCADE"))
            logger.info("Cleaned up any conflicting 'missions' type if it existed")
    except Exception as e:
        # If we can't drop the type, log a warning but continue
        # This might happen if the type doesn't exist (which is fine) or if there are permission issues
        logger.debug(f"Note while cleaning up types: {e}. This is usually fine if type doesn't exist.")
    
    # Create tables
    try:
        Base.metadata.create_all(bind=engine)
    except ProgrammingError as e:
        # If there's still a conflict, try to handle it more gracefully
        if "duplicate key value violates unique constraint" in str(e) or "pg_type_typname_nsp_index" in str(e):
            logger.error(f"Database type conflict: {e}")
            logger.error("Please manually clean up the database or use a different database name.")
            raise RuntimeError(
                "Database initialization failed due to conflicting type 'missions'. "
                "This can happen if a composite type or enum with this name exists. "
                "Try running: DROP TYPE IF EXISTS missions CASCADE; in your PostgreSQL database."
            ) from e
        raise


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

