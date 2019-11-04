"""Time values and functions."""

from __future__ import annotations

Timestamp = float
Duration = float

SECOND = 1.0
MINUTE = 60.0 * SECOND
HOUR = 60.0 * MINUTE
DAY = 24.0 * HOUR


def normalized_hours(timestamp: Timestamp) -> float:
    """Get hours in range 0.0 <= hours < 24.0 from timestamp."""
    return (timestamp % DAY) / HOUR


def whole_days(timestamp: Timestamp) -> int:
    """Get whole number of days since epoch from timestamp."""
    return timestamp // DAY
