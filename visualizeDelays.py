import sys
import matplotlib.pyplot as plt
import numpy as np
with open(sys.argv[1], "r") as f:
    lines = f.readlines()
    delays = [int(line) for line in lines[:-1]]
    good,bad = [int(val) for val in lines[-1].split()]
    delaysU, counts = np.unique(delays, return_counts=True)
    counts = np.cumsum(counts, dtype=float)
    counts /= counts[-1]
    plt.plot(delaysU, counts)
    plt.show()
    print(good,bad)
    
