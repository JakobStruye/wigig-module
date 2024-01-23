import sys
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation


colors = plt.rcParams['axes.prop_cycle'].by_key()['color']

def angle_between_yaw_roll_pitch(yaw_roll_pitch_1, yaw_roll_pitch_2):
  # Convert Yaw-Roll-Pitch angles to rotation matrices (use 'ZXY' order)
  rotation_matrix_1 = Rotation.from_euler('ZXY', yaw_roll_pitch_1, degrees=False).as_matrix()
  rotation_matrix_2 = Rotation.from_euler('ZXY', yaw_roll_pitch_2, degrees=False).as_matrix()

  # Calculate the rotation matrix representing the rotation from the first to the second
  relative_rotation_matrix = np.dot(rotation_matrix_1.T, rotation_matrix_2)

  # Extract the rotation axis and angle from the matrix
  rotvec = Rotation.from_matrix(relative_rotation_matrix).as_rotvec(degrees=True)
  angle = np.linalg.norm(rotvec)

  return angle
for filename in sys.argv[1:]:
  with open(filename, "r") as f:
    lines = f.readlines()
    orients = [line.split(",") for line in lines[:]]
    orients = [[float(o) for o in orient] for orient in orients]

    angles = []
    for i in range(len(orients)-100):
      angles.append(angle_between_yaw_roll_pitch(orients[i+100], orients[i]))
    angles = [angle * 10.0 for angle in angles]
    angles = sorted(angles)
    plt.plot(angles, np.array(range(len(angles)))/len(angles))

    print("Max", angles[-1])
    print("Median", angles[len(angles)//2])
    print("Mean", np.sum(angles) / len(angles))
    print("95th", angles[int(len(angles) * 0.95)])


plt.xlabel("Rot speed")
plt.ylabel("CDF")
plt.legend()
plt.show()
#plt.savefig(sys.argv[-1] + ".pdf")
