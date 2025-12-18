from pydantic import BaseModel
from typing import Optional
from datetime import datetime


class UserFeedbackRequest(BaseModel):
    """
    Request model for user feedback operations.
    """
    interaction_id: str  # Reference to the chat interaction
    rating: Optional[int] = None  # Numerical rating (1-5 scale)
    thumbs: Optional[str] = None  # "up", "down", or null
    comment: Optional[str] = None  # Optional textual feedback
    useful: Optional[bool] = None  # Whether the response was helpful
    accuracy_rating: Optional[int] = None  # Accuracy rating (1-5 scale)


class UserFeedback(BaseModel):
    """
    Model representing user feedback.
    """
    id: str  # Unique identifier for the feedback
    interaction_id: str  # Reference to the chat interaction
    rating: Optional[int] = None  # Numerical rating (1-5 scale)
    thumbs: Optional[str] = None  # "up", "down", or null
    comment: Optional[str] = None  # Optional textual feedback
    useful: Optional[bool] = None  # Whether the response was helpful
    accuracy_rating: Optional[int] = None  # Accuracy rating (1-5 scale)
    timestamp: datetime  # When feedback was provided


class UserFeedbackResponse(BaseModel):
    """
    Response model for user feedback operations.
    """
    feedback_id: str  # ID of the created feedback
    status: str  # Status of the feedback submission ("success", "error")
    message: str  # Additional information about the feedback submission


class FeedbackSummary(BaseModel):
    """
    Model for feedback summary statistics.
    """
    total_feedback: int  # Total number of feedback entries
    avg_rating: float  # Average numerical rating (1-5 scale)
    thumbs_up_percentage: float  # Percentage of thumbs up (0-1)
    thumbs_down_percentage: float  # Percentage of thumbs down (0-1)
    useful_percentage: float  # Percentage marked as useful (0-1)
    avg_accuracy_rating: float  # Average accuracy rating (1-5 scale)
    top_comments: list[str]  # Most common feedback comments
    time_period: str  # Time period for the summary (e.g., "last_week", "last_month")