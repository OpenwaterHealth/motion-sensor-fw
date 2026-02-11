#!/usr/bin/env python3
"""
merge_hex.py - Merge two Intel HEX files into one.

Usage: merge_hex.py <output.hex> <input1.hex> <input2.hex> [input3.hex ...]

Concatenates multiple Intel HEX files, stripping intermediate EOF records.
"""

import sys
import os

def merge_hex_files(output_path, input_paths):
    """Merge multiple Intel HEX files into a single file."""
    EOF_RECORD = ":00000001FF"
    
    with open(output_path, 'w') as out:
        for i, path in enumerate(input_paths):
            if not os.path.exists(path):
                print(f"Error: File not found: {path}", file=sys.stderr)
                sys.exit(1)
            
            with open(path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    # Skip EOF records from all but the last file
                    if line.upper() == EOF_RECORD and i < len(input_paths) - 1:
                        continue
                    out.write(line + '\n')
        
        # Ensure we end with an EOF record
        # Check if the last file ended with one
        with open(input_paths[-1], 'r') as f:
            last_lines = [l.strip().upper() for l in f.readlines() if l.strip()]
            if not last_lines or last_lines[-1] != EOF_RECORD:
                out.write(EOF_RECORD + '\n')

    print(f"Merged {len(input_paths)} hex files -> {output_path}")

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print(f"Usage: {sys.argv[0]} <output.hex> <input1.hex> <input2.hex> [...]")
        sys.exit(1)
    
    output = sys.argv[1]
    inputs = sys.argv[2:]
    merge_hex_files(output, inputs)
