#!/usr/bin/python
import argparse
import glob
import sys

print("=" * 60)
print("DEBUG TEST - Instance File Loading")
print("=" * 60)

# Print Python arguments
print(f"\nPython Version: {sys.version}")
print(f"Command line arguments: {sys.argv}")

# Test argparse
parser = argparse.ArgumentParser(description='Debug instance loading')
parser.add_argument('--instance', type=str, default=None,
                    help='The name of the instance file(s)')
parser.add_argument('--solver', type=str, default="CBS",
                    help='The solver to use')
parser.add_argument('--batch', action='store_true', default=False,
                    help='Use batch output')

args = parser.parse_args()

print(f"\nParsed arguments:")
print(f"  args.instance = {repr(args.instance)}")
print(f"  args.solver = {repr(args.solver)}")
print(f"  args.batch = {repr(args.batch)}")

# Test glob pattern
if args.instance:
    print(f"\nSearching for instances matching: {repr(args.instance)}")
    matches = sorted(glob.glob(args.instance))
    print(f"Found {len(matches)} files:")
    for i, file in enumerate(matches, 1):
        print(f"  {i}. {file}")
else:
    print("\nERROR: --instance is None!")

print("\n" + "=" * 60)
