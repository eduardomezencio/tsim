"""Units, values and functions for space and time."""

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


def kph_to_mps(speed_kph: float) -> float:
    """Convert kilometers per hour to meters per second."""
    return 1000.0 * speed_kph / HOUR


def mps_tp_kph(speed_mps: float) -> float:
    """Convert meters per second to kilometers per hour."""
    return HOUR * speed_mps / 1000.0
