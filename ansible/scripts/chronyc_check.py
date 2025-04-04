#!/bin/python3

import subprocess
import re

sp = subprocess.run(["chronyc", "tracking"], capture_output=True)
output = sp.stdout.decode('utf-8')

ref_id = re.search("Reference ID\\s+:\\s+([A-Z0-9]{8})", output).group(1)

print(f"Found Reference ID: {ref_id}")
if not ref_id or ref_id == "00000000":
    exit(1)

exit(0)