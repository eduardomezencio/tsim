"""Time values and functions.

Time is stored as an integer, representing microseconds from some undefined
epoch.
"""

MICROSECOND = 1
MILLISECOND = 1000 * MICROSECOND
SECOND = 1000 * MILLISECOND
MINUTE = 60 * SECOND
HOUR = 60 * MINUTE
DAY = 24 * HOUR


def normalized_hours(timestamp: int) -> float:
    """Get hours in range 0.0 <= hours < 24.0 from timestamp."""
    return (timestamp % DAY) / HOUR


def whole_days(timestamp: int) -> int:
    """Get whole number of days since epoch from timestamp."""
    return timestamp // DAY
