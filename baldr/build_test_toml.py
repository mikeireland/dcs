#!/usr/bin/env python3
import toml
import sys

def generate_config(beam_id, phasemask):
    config = {}

    # Global configuration
    config["baldr_pupils"] = {
        "pupil1": [1, 2, 3],
        "pupil2": [4, 5, 6]
    }

    # Beam-specific configuration (e.g., "beam2")
    beam_key = f"beam{beam_id}"
    config[beam_key] = {}

    # I2A: 2D array of numbers
    config[beam_key]["I2A"] = [
        [1.1, 2.2],
        [3.3, 4.4]
    ]

    # Image pixel filters under "pupil_mask"
    config[beam_key]["pupil_mask"] = {
        "mask": [[True, False], [False, True]],
        "exterior": [[False, True], [True, False]],
        "secondary": [[True, True], [False, False]]
    }

    # Control model under the phase mask key
    config[beam_key][phasemask] = {}
    config[beam_key][phasemask]["ctrl_model"] = {
        "IM": [[0.1, 0.2], [0.3, 0.4]],
        "I2M": [[1.0, 2.0], [3.0, 4.0]],
        "M2C": [[5.0, 6.0], [7.0, 8.0]],
        "I0": [[9.0, 10.0], [11.0, 12.0]],
        "N0": [[13.0, 14.0], [15.0, 16.0]],
        "norm_pupil": [[17.0, 18.0], [19.0, 20.0]],
        "inner_pupil_filt": [[True, False], [False, True]],
        "camera_config": {
            "fps": 0.033,
            "gain": 1.5
        },
        "bad_pixel_mask": [[False, True], [True, False]],
        "bias": [[100.0, 101.0], [102.0, 103.0]],
        "dark": [[0.1, 0.1], [0.1, 0.1]]
    }

    # Strehl model configuration under the beam section
    config[beam_key]["strehl_model"] = {
        "secondary": [[1.0, 0.5]],
        "exterior": [[2.0, 1.0]]
    }

    return config

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python generate_config.py <beam_id> <phasemask>")
        sys.exit(1)
    
    beam_id = sys.argv[1]
    phasemask = sys.argv[2]
    config = generate_config(beam_id, phasemask)
    
    filename = f"baldr_config_{beam_id}.toml"
    with open(filename, "w") as f:
        toml.dump(config, f)
    
    print(f"Config file generated: {filename}")