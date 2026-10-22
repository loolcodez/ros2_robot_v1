import sys
from unittest.mock import MagicMock

# Create fake modules globally for the test run. This prevents ModuleNotFoundError coming
sys.modules['periphery'] = MagicMock()
sys.modules['VL53L1X'] = MagicMock()
