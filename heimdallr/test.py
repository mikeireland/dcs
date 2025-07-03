data = """
Baseline: 0 x_px_K1: 0.212114 y_px_K1: 12.3758
Baseline: 0 x_px_K2: 0.19367 y_px_K2: 11.2996
Baseline: 1 x_px_K1: 3.10857 y_px_K1: 5.4272
Baseline: 1 x_px_K2: 2.83826 y_px_K2: 4.95527
Baseline: 2 x_px_K1: 10.8983 y_px_K1: 5.37234
Baseline: 2 x_px_K2: 9.95061 y_px_K2: 4.90518
Baseline: 3 x_px_K1: 2.89646 y_px_K1: 25.0514
Baseline: 3 x_px_K2: 2.64459 y_px_K2: 25.6557
Baseline: 4 x_px_K1: 10.6862 y_px_K1: 24.9966
Baseline: 4 x_px_K2: 9.75694 y_px_K2: 25.6056
Baseline: 5 x_px_K1: 7.78971 y_px_K1: 31.9451
Baseline: 5 x_px_K2: 7.11235 y_px_K2: 31.9499
"""

# Convert the data to a dictionary
result = {}
for line in data.strip().split("\n"):
    parts = line.split()
    baseline = int(parts[1])
    key = parts[2]
    x_value = float(parts[3])
    y_key = parts[4]
    y_value = float(parts[5])
    
    if baseline not in result:
        result[baseline] = {}
    result[baseline][key] = {"x": x_value, "y": y_value}

print(result)
y2 = [result[j]["x_px_K2:"]["y"] for j in range(6)]
x2 = [result[j]["x_px_K2:"]["x"] for j in range(6)]
y1 = [result[j]["x_px_K1:"]["y"] for j in range(6)]
x1 = [result[j]["x_px_K1:"]["x"] for j in range(6)]

