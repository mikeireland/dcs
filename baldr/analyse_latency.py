# Write the analyze_latency.py script to a file for the user
#!/usr/bin/env python
import json
import numpy as np
import argparse
import matplotlib.pyplot as plt

def analyze_latency(json_file, threshold_factor=3, pre_poke_window=5):
    # Load JSON
    with open(json_file, 'r') as f:
        data = json.load(f)
    timestamps = np.array(data['timestamps_ms'])
    poke_vals = np.array(data['poke_values'])
    images = np.array(data['images'])  # shape (n_frames, n_pixels)

    # Compute mean intensity per frame
    mean_intensity = images.mean(axis=1)

    # Identify poke ON phases: when poke_vals == poke_amp
    poke_on = np.where((poke_vals > 0))[0]

    # Find the start of each ON block
    starts = poke_on[np.where(np.diff(np.concatenate(([0], poke_on))) > 1)]

    delays = []
    for onset in starts:
        # Define baseline from pre_poke_window before onset
        start = max(0, onset - pre_poke_window)
        baseline = mean_intensity[start:onset]
        mu, sigma = baseline.mean(), baseline.std()
        threshold = mu + threshold_factor * sigma

        # Search for first frame after onset exceeding threshold
        post = mean_intensity[onset:]
        above = np.where(post > threshold)[0]
        if above.size > 0:
            delay = above[0]
            delays.append(delay)

    delays = np.array(delays)
    print(f"Detected {len(delays)} poke events.")
    if len(delays) == 0:
        print("No events exceeded threshold.")
        return

    # Output statistics
    dt = np.mean(np.diff(timestamps))  # average ms per frame
    print(f"Delays (frames): mean={delays.mean():.2f}, std={delays.std():.2f}")
    print(f"Delays (ms): mean={delays.mean()*dt:.2f}, std={delays.std()*dt:.2f}")

    # Plot histogram
    plt.figure()
    plt.hist(delays, bins=range(0, int(delays.max())+2), align='left', rwidth=0.8)
    plt.xlabel('Delay (frames)')
    plt.ylabel('Count')
    plt.title('Latency Histogram')
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze latency from latency_telem.json")
    parser.add_argument("json_file", help="Path to latency_telem.json")
    parser.add_argument("--threshold_factor", type=float, default=3.0, 
                        help="Number of baseline std devs for detection (default: 3)")
    parser.add_argument("--pre_poke_window", type=int, default=5,
                        help="Number of frames before poke to compute baseline (default: 5)")
    args = parser.parse_args()

    analyze_latency(args.json_file, args.threshold_factor, args.pre_poke_window)
