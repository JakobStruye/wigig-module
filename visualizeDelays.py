import sys
import matplotlib.pyplot as plt
import numpy as np

colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
plt.rcParams['figure.figsize'] = (6.4, 4)

inputs_count = (len(sys.argv)-2) // 2
# labels=["CoVRage", "Big QO", "Big QO (az only)", "Small QO", "Small QO (az only)", "Small sectors"]
# labels=["CoVRage (on-device pred.)", "CoVRage (model-basec pred.)", "CoVRage (oracle)", "Quasi-Omni", "Quasi-Omni (small array)", "Sector-based"]
labels = sys.argv[inputs_count+1:-1]
trace_idx = 0
inputs_legended = inputs_count if inputs_count < 6 else inputs_count//2
thresh = 1
title="No timeout, loss threshold 1.0"
for filename in sys.argv[1:inputs_count+1]:
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
    good_with_thresh = counts[-1]
    counts /= good_with_thresh
    rcvdfrac = good_with_thresh / (good+bad)
    counts *= rcvdfrac
    plt.plot(delaysU, counts, label=labels[trace_idx] if trace_idx < inputs_legended else None, color=colors[trace_idx%inputs_legended],
             ls="-" if trace_idx < inputs_legended else "--")
    if delaysU[-1] > 20:
      idx_pos = (np.abs(delaysU - 19)).argmin()
      idx_val = (np.abs(delaysU - 20)).argmin()
      plt.text(18, counts[idx_pos]-0.01, "{:.2f}%".format(100.0*counts[idx_val]), color=colors[trace_idx%inputs_legended],
               backgroundcolor="white", fontsize=8, zorder=10,
               bbox=dict(facecolor="white", edgecolor='none', boxstyle='round', pad=0.1))
    trace_idx+=1
    print(good,bad)
plt.ylim(-0.05, 1.05)
plt.xlim(0,20)
plt.xlabel("Image delay (ms)")
plt.ylabel("CDF")
plt.legend()
# plt.show()
plt.savefig(sys.argv[-1] + ".pdf")