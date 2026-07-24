"""Allow the package to be executed with ``python -m adaptive_assist``."""

from adaptive_assist.main import main

if __name__ == "__main__":
    raise SystemExit(main())
