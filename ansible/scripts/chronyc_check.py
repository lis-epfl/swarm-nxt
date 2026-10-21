#!/bin/python3

import subprocess
import re

sp = subprocess.run(["chronyc", "tracking"], capture_output=True)
output = sp.stdout.decode('utf-8')

ref_m = re.search(r"Reference ID\s+:\s+([A-F0-9]{8})", output)
if not ref_m:
    print("Did not get a Reference ID...")
    exit(1)
ref_id = ref_m.group(1)
ref_int = int(ref_id, 16)
print(f"Found Reference ID: 0x{ref_int:X}")
if not ref_int:
    exit(1)

# check rms offset
rms_m = re.search(r"RMS offset\s+:\s+([0-9]+\.[0-9]+) seconds", output)
if not rms_m:
    print("Did not get an RMS offset...")
    exit(1)
rms_offset = float(rms_m.group(1))
print(f"RMS Offset: {rms_offset}")
if rms_offset > 0.005:
    print("Too slow!")
    exit(1)

exit(0)
