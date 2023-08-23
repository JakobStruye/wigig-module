import sys
import matplotlib.pyplot as plt
import numpy as np
for filename in sys.argv[1:]:
  with open(filename, "r") as f:
    lines = f.readlines()
    delays = [int(line) for line in lines[:-1]]
    good,bad = [int(val) for val in lines[-1].split()]
    delaysU, counts = np.unique(delays, return_counts=True)
    counts = np.cumsum(counts, dtype=float)
    counts /= counts[-1]
    rcvdfrac = good / (good+bad)
    counts *= rcvdfrac
    plt.plot(delaysU, counts)
    print(good,bad)
plt.ylim(-0.05, 1.05)
plt.show()
    
