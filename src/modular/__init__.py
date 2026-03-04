import logging

# Standard library practice: add a NullHandler so log records are silently
# discarded when the application has not configured logging yet.  This avoids
# the "No handlers could be found" warning that Python emits for unconfigured
# loggers in library code.
logging.getLogger(__name__).addHandler(logging.NullHandler())

# ---------------------------------------------------------------------------
# The canonical log format for the modular package.
# It mimics the ROS console format so output looks familiar in ROS workflows,
# but the constant lives here so it is defined exactly once.
# ---------------------------------------------------------------------------
LOG_FORMAT = '[%(levelname)s] [%(module)s]:  %(message)s'


def setup_logging(level: int = logging.INFO) -> None:
    """Configure the root logger with the standard modular log format.

    Call this **once** from an application entry-point or script — never from
    library code.  It is a thin wrapper around ``logging.basicConfig`` that
    makes the canonical format available without requiring every script to
    define its own FORMAT string.

    Parameters
    ----------
    level:
        Minimum log level to display (default ``logging.INFO``).
        Pass ``logging.DEBUG`` to see other verbose diagnostics.
    """
    logging.basicConfig(level=level, format=LOG_FORMAT)
