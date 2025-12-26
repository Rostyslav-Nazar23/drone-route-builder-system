"""Run tests for the system."""
import unittest
import sys
from pathlib import Path

# Add project root to path
project_root = Path(__file__).parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

# Discover and run tests
loader = unittest.TestLoader()
suite = loader.discover('tests', pattern='test_*.py')

runner = unittest.TextTestRunner(verbosity=2)
result = runner.run(suite)

# Exit with appropriate code
sys.exit(0 if result.wasSuccessful() else 1)

