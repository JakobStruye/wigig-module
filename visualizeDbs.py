import sys
import matplotlib.pyplot as plt
import numpy as np
for filename in sys.argv[1:]:
  with open(filename, "r") as f:
    lines = f.readlines()
    dbs = [float(line.split()[1]) for line in lines]
    dbsU, counts = np.unique(dbs, return_counts=True)
    counts = np.cumsum(counts, dtype=float)
    counts /= counts[-1]
    plt.plot(dbsU, counts)
plt.xlabel("Gain (dBW)")
plt.ylabel("CDF")
plt.ylim(-0.05, 1.05)
plt.show()
    
