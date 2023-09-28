import sys
import matplotlib.pyplot as plt
import numpy as np
labels=["CoVRage", "Big QO", "Big QO (az only)", "Small QO", "Small QO (az only)", "Small sectors"]
label_idx = 0
thresh = 0.85
for filename in sys.argv[1:]:
  with open(filename, "r") as f:
    lines = f.readlines()
    delays = [int(line.split()[0]) for line in lines[:-1]]
    frac_per_frame = [float(line.split()[1]) for line in lines[:-1]]
    delays = [delays[i] for i in range(len(delays)) if frac_per_frame[i] >= thresh]
    delays = np.array(delays, dtype=float)
    delays /= 1000
    good,bad = [int(val) for val in lines[-1].split()]
    delaysU, counts = np.unique(delays, return_counts=True)
    print("LEN", len(delaysU), len(delays), max(frac_per_frame), max(frac_per_frame) >= thresh)
    counts = np.cumsum(counts, dtype=float)
    counts /= counts[-1]
    rcvdfrac = good / (good+bad)
    counts *= rcvdfrac
    plt.plot(delaysU, counts, label=labels[label_idx])
    label_idx+=1
    print(good,bad)
plt.ylim(-0.05, 1.05)
plt.xlabel("Video frame delay (ms)")
plt.ylabel("CDF")
plt.legend()
plt.show()
    
