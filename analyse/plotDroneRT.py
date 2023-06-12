import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import pdb # for debugging
from matplotlib import animation

# Read positions from the file
positions = {}
time = {}
with open("drone_positions.txt", "r") as file:
    for line in file:
        t, drone_id, x, y, z = map(float, line.strip().split())
        if drone_id not in positions:
            positions[drone_id] = []
        positions[drone_id].append([x, y, z, t])

# Create a 3D plot
fig = plt.figure(figsize=(8, 6))
#ax = fig.add_subplot(221, projection='3d')

# Plot every x-th point
every_xth_data = 100

# XZ plot
ax_xz = fig.add_subplot(111)
for drone_id, drone_positions in positions.items():
    x = [position[0] for position in drone_positions[::every_xth_data]]
    z = [-position[2] for position in drone_positions[::every_xth_data]]  # z comes in NED. Revert sign for negative down, positive up
    #pdb.set_trace()
    line, = ax_xz.plot(x, z, 'o-', label=f"Drone {int(drone_id)}")
    def connect(i):
        start=max((i-3,0))
        line.set_data(x[start:i],z[start:i])
        return line,
    ani = animation.FuncAnimation(fig, connect, np.arange(1, 100), interval=100)
plt.show()
ax_xz.set_xlabel('X [m]')
ax_xz.set_ylabel('Z [m]')
ax_xz.set_title('XZ Plane')
ax_xz.legend()