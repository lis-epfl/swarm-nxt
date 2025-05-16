#!/bin/python3

import subprocess
import re

sp = subprocess.run(["chronyc", "tracking"], capture_output=True)
output = sp.stdout.decode('utf-8')

ref_id = re.search("Reference ID\\s+:\\s+([A-F0-9]{8})", output).group(1)
if ref_id: 
    ref_id = int(ref_id, 16)

print(f"Found Reference ID: 0x{ref_id:X}")
if not ref_id:
    exit(1)

# check rms offset
rms_offset = re.search("RMS offset\\s+:\\s+([0-9]?\\.[0-9]+) seconds", output).group(1)
if not rms_offset: 
    print("Did not get an RMS offset...")
    exit(1)

rms_offset = float(rms_offset)
print(f"RMS Offset: {rms_offset}")
if rms_offset > 0.005: 
    print("Too slow!")
    exit(1)

exit(0)