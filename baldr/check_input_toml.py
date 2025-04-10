#!/usr/bin/env python3
import re
import sys
import os
import toml

def determine_dimensions(obj):
    """
    For a list, return 1 if flat (1D) or 2 if it is a list of lists (2D).
    """
    if not isinstance(obj, list):
        return None
    if len(obj) == 0:
        return 1
    if isinstance(obj[0], list):
        return 2
    return 1

def get_structure_signature(obj):
    """
    Recursively compute a structure signature for the given object.
    
    - If the object is a dict, return a dict mapping keys to signatures.
    - If it is a list, return a tuple ("list", dimension, signature of first element or None).
    - For numbers, treat int and float as the same ("number").
    - Otherwise, return the type name.
    """
    if isinstance(obj, dict):
        return {k: get_structure_signature(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        dim = determine_dimensions(obj)
        if len(obj) == 0:
            inner = None
        elif dim == 1:
            inner = get_structure_signature(obj[0])
        elif dim == 2:
            inner = get_structure_signature(obj[0][0]) if obj[0] else None
        return ("list", dim, inner)
    else:
        if isinstance(obj, (int, float)):
            return "number"
        return type(obj).__name__

def compare_signatures(sig1, sig2, path=""):
    """
    Recursively compare two structure signatures.
    
    Returns a list of error messages; an empty list means the signatures match.
    """
    errors = []
    if isinstance(sig1, dict) and isinstance(sig2, dict):
        for k in sig1:
            new_path = f"{path}.{k}" if path else k
            if k not in sig2:
                errors.append(f"Missing key '{new_path}' in candidate configuration")
            else:
                errors.extend(compare_signatures(sig1[k], sig2[k], new_path))
        for k in sig2:
            if k not in sig1:
                new_path = f"{path}.{k}" if path else k
                errors.append(f"Extra key '{new_path}' found in candidate configuration")
    else:
        if sig1 != sig2:
            errors.append(f"Type/dimension mismatch at '{path}': expected {sig1}, got {sig2}")
    return errors

def get_first_beam_config(config):
    """
    Look for the first key in the config that matches 'beam\d+'.
    Return the nested configuration under that key.
    """
    for key in config:
        if re.match(r'^beam\d+', key):
            return config[key]
    return None

def validate_two_files(ref_file, candidate_file):
    """
    Load two TOML files, extract their beam configuration (ignoring the beam number)
    and compare their structure.
    Returns a list of errors (empty if compatible).
    """
    if not os.path.exists(ref_file):
        return [f"Reference file not found: {ref_file}"]
    if not os.path.exists(candidate_file):
        return [f"Candidate file not found: {candidate_file}"]
    
    ref_config = toml.load(ref_file)
    cand_config = toml.load(candidate_file)
    
    ref_beam = get_first_beam_config(ref_config)
    if ref_beam is None:
        return [f"No beam configuration (e.g., 'beam1') found in reference file."]
    
    cand_beam = get_first_beam_config(cand_config)
    if cand_beam is None:
        return [f"No beam configuration (e.g., 'beam1') found in candidate file."]
    
    ref_sig = get_structure_signature(ref_beam)
    cand_sig = get_structure_signature(cand_beam)
    
    errors = compare_signatures(ref_sig, cand_sig, "beam")
    return errors

def main():
    if len(sys.argv) != 3:
        print("Usage: check_two_beam_toml.py <reference_file.toml> <candidate_file.toml>")
        sys.exit(1)
    
    ref_file = sys.argv[1]
    candidate_file = sys.argv[2]
    
    errors = validate_two_files(ref_file, candidate_file)
    if errors:
        print("Structural incompatibilities found:")
        for err in errors:
            print(" -", err)
        sys.exit(1)
    else:
        print("The candidate configuration is structurally compatible with the reference.")

if __name__ == "__main__":
    main()


#  python check_input_toml.py /home/asg/Progs/repos/dcs/baldr/baldr_config_1.toml /home/asg/Progs/repos/dcs/baldr/baldr_config_2.toml