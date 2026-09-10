"""Command-line entry point for the current project status."""

FOUNDATION_MESSAGE = (
    "adaptive-assist includes a deterministic 1-DOF joint model and open-loop "
    "experiment interfaces; controllers and safety supervision are not yet "
    "implemented."
)


def main() -> int:
    """Print the current project status and return a successful exit code."""
    print(FOUNDATION_MESSAGE)
    return 0
