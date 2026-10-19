"""Allow legacy checkout scripts without requiring an editable installation."""
from pathlib import Path
import sys

_source = str(Path(__file__).resolve().parent / "src")
if _source not in sys.path:
    sys.path.insert(0, _source)
