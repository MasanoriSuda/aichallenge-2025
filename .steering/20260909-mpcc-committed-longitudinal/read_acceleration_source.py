"""Read the used container's acceleration producer and its low-pass implementation."""
from pathlib import Path
for root in [Path('/autoware'),Path('/opt/autoware')]:
 for pattern in ['*twist2accel*.cpp','*lowpass_filter*.hpp']:
  for p in root.rglob(pattern):
   print(str(p),flush=True)
   print(p.read_text(),flush=True)
