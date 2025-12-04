#!/usr/bin/env python3
"""
Convert VTOL position controller parameters from NEU to NED naming convention.

This script converts parameter files (.parm, .param, .params) from the old
NEU-based naming (PSC_ACCZ_*, PSC_VELZ_*, etc.) to the new NED-based naming
(PSC_D_ACC_*, PSC_D_VEL_*, etc.) with appropriate value scaling.

Conversion rules:
- PSC_ACCZ_* -> PSC_D_ACC_*
- PSC_VELZ_* -> PSC_D_VEL_*
- PSC_POSZ_* -> PSC_D_POS_*
- PSC_VELXY_* -> PSC_NE_VEL_*
- PSC_POSXY_* -> PSC_NE_POS_*
- PSC_JERK_Z -> PSC_JERK_D
- PSC_JERK_XY -> PSC_JERK_NE
- Q_P_* versions follow the same pattern

Value scaling:
- _ACC_P, _ACC_I, _ACC_D: multiply by 0.1
- _VEL_IMAX: multiply by 0.01
- _ACC_IMAX: multiply by 0.001
- All other parameters: no scaling
"""

import argparse
import os
import re
import sys

# Name conversions: (old_pattern, new_pattern)
# Order matters - more specific patterns first
NAME_CONVERSIONS = [
    # Copter/Sub PSC parameters
    (r'^PSC_ACCZ_', 'PSC_D_ACC_'),
    (r'^PSC_VELZ_', 'PSC_D_VEL_'),
    (r'^PSC_POSZ_', 'PSC_D_POS_'),
    (r'^PSC_VELXY_', 'PSC_NE_VEL_'),
    (r'^PSC_POSXY_', 'PSC_NE_POS_'),
    (r'^PSC_JERK_Z$', 'PSC_JERK_D'),
    (r'^PSC_JERK_XY$', 'PSC_JERK_NE'),
    # QuadPlane Q_P_ parameters
    (r'^Q_P_ACCZ_', 'Q_P_D_ACC_'),
    (r'^Q_P_VELZ_', 'Q_P_D_VEL_'),
    (r'^Q_P_POSZ_', 'Q_P_D_POS_'),
    (r'^Q_P_VELXY_', 'Q_P_NE_VEL_'),
    (r'^Q_P_POSXY_', 'Q_P_NE_POS_'),
    (r'^Q_P_JERK_Z$', 'Q_P_JERK_D'),
    (r'^Q_P_JERK_XY$', 'Q_P_JERK_NE'),
]

# Value scaling rules: suffix -> scale factor
# These are applied to the NEW parameter names after conversion
VALUE_SCALING = {
    '_ACC_P': 0.1,
    '_ACC_I': 0.1,
    '_ACC_D': 0.1,
    '_VEL_IMAX': 0.01,
    '_ACC_IMAX': 0.001,
}


def convert_param_name(name):
    """Convert old parameter name to new name. Returns (new_name, was_converted)."""
    for pattern, replacement in NAME_CONVERSIONS:
        if re.match(pattern, name):
            new_name = re.sub(pattern, replacement, name)
            return new_name, True
    return name, False


def get_scale_factor(new_name):
    """Get the scale factor for a parameter based on its new name."""
    for suffix, scale in VALUE_SCALING.items():
        if new_name.endswith(suffix):
            return scale
    return 1.0


def parse_param_line(line):
    """
    Parse a parameter line and return (prefix, name, separator, value, suffix).
    Returns None if line is not a parameter line.

    Handles formats:
    - "PARAM_NAME,value" or "PARAM_NAME, value"
    - "PARAM_NAME value" or "PARAM_NAME    value"
    - "1 1 PARAM_NAME value 9" (QGC format)
    """
    line_stripped = line.strip()

    # Skip empty lines and comments
    if not line_stripped or line_stripped.startswith('#'):
        return None

    # Try QGC format: "1 1 PARAM_NAME value 9"
    qgc_match = re.match(r'^(\d+\s+\d+\s+)(\S+)(\s+)(\S+)(\s+\d+.*)$', line_stripped)
    if qgc_match:
        return qgc_match.groups()

    # Try comma-separated format: "PARAM_NAME,value" or "PARAM_NAME, value"
    comma_match = re.match(r'^(\s*)([A-Z][A-Z0-9_]*)(,\s*)(\S+)(\s*)$', line)
    if comma_match:
        return comma_match.groups()

    # Try space/tab separated format: "PARAM_NAME value"
    space_match = re.match(r'^(\s*)([A-Z][A-Z0-9_]*)(\s+)(\S+)(\s*)$', line)
    if space_match:
        return space_match.groups()

    return None


def convert_value(value_str, scale):
    """Convert a value string by applying scale factor."""
    if scale == 1.0:
        return value_str

    try:
        value = float(value_str)
        new_value = value * scale

        # Preserve reasonable precision
        if new_value == int(new_value):
            return str(int(new_value))
        else:
            # Format to avoid excessive decimal places
            formatted = f"{new_value:.10g}"
            return formatted
    except ValueError:
        # Not a number, return unchanged
        return value_str


def detect_line_ending(filepath):
    """Detect line ending style (CRLF or LF) by reading file in binary mode."""
    try:
        with open(filepath, 'rb') as f:
            content = f.read(8192)  # Read first 8KB to detect
            if b'\r\n' in content:
                return '\r\n'  # DOS/Windows CRLF
            return '\n'  # Unix LF
    except Exception:
        return '\n'  # Default to Unix


def convert_file(filepath, dry_run=False, verbose=False):
    """
    Convert a single parameter file.
    Returns (num_conversions, error_message or None).
    """
    # Detect original line ending style before reading
    line_ending = detect_line_ending(filepath)

    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()
    except Exception as e:
        return 0, f"Error reading file: {e}"

    new_lines = []
    num_conversions = 0

    for line_num, line in enumerate(lines, 1):
        parsed = parse_param_line(line)

        if parsed is None:
            # Not a parameter line, keep as-is
            new_lines.append(line)
            continue

        prefix, name, separator, value, suffix = parsed

        # Convert parameter name
        new_name, was_converted = convert_param_name(name)

        if was_converted:
            # Get scale factor and convert value
            scale = get_scale_factor(new_name)
            new_value = convert_value(value, scale)

            # Reconstruct line
            new_line = f"{prefix}{new_name}{separator}{new_value}{suffix}"
            if not new_line.endswith('\n') and line.endswith('\n'):
                new_line += '\n'

            new_lines.append(new_line)
            num_conversions += 1

            if verbose:
                scale_info = f" (×{scale})" if scale != 1.0 else ""
                print(f"  {filepath}:{line_num}: {name}={value} -> {new_name}={new_value}{scale_info}")
        else:
            new_lines.append(line)

    if num_conversions > 0 and not dry_run:
        try:
            # Write with original line ending style
            with open(filepath, 'w', newline=line_ending) as f:
                f.writelines(new_lines)
        except Exception as e:
            return num_conversions, f"Error writing file: {e}"

    return num_conversions, None


def find_param_files(root_dir):
    """Find all .parm, .param, and .params files in directory tree."""
    param_files = []
    for dirpath, dirnames, filenames in os.walk(root_dir):
        # Skip hidden directories and build directories
        dirnames[:] = [d for d in dirnames if not d.startswith('.') and d != 'build']

        for filename in filenames:
            if filename.endswith(('.parm', '.param', '.params')):
                param_files.append(os.path.join(dirpath, filename))

    return sorted(param_files)


def main():
    parser = argparse.ArgumentParser(
        description='Convert VTOL position controller parameters from NEU to NED naming convention.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        'paths',
        nargs='*',
        default=['.'],
        help='Files or directories to process (default: current directory)'
    )
    parser.add_argument(
        '-n', '--dry-run',
        action='store_true',
        help='Show what would be changed without modifying files'
    )
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Show each parameter conversion'
    )
    parser.add_argument(
        '-q', '--quiet',
        action='store_true',
        help='Only show errors'
    )

    args = parser.parse_args()

    # Collect all files to process
    files_to_process = []
    for path in args.paths:
        if os.path.isfile(path):
            files_to_process.append(path)
        elif os.path.isdir(path):
            files_to_process.extend(find_param_files(path))
        else:
            print(f"Warning: {path} not found", file=sys.stderr)

    if not files_to_process:
        print("No parameter files found", file=sys.stderr)
        return 1

    total_files = 0
    total_conversions = 0
    errors = []

    for filepath in files_to_process:
        num_conversions, error = convert_file(filepath, args.dry_run, args.verbose)

        if error:
            errors.append(f"{filepath}: {error}")
        elif num_conversions > 0:
            total_files += 1
            total_conversions += num_conversions
            if not args.quiet and not args.verbose:
                action = "Would convert" if args.dry_run else "Converted"
                print(f"{action} {num_conversions} parameters in {filepath}")

    # Summary
    if not args.quiet:
        print()
        if args.dry_run:
            print(f"Dry run: would convert {total_conversions} parameters in {total_files} files")
        else:
            print(f"Converted {total_conversions} parameters in {total_files} files")

    if errors:
        print("\nErrors:", file=sys.stderr)
        for error in errors:
            print(f"  {error}", file=sys.stderr)
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
