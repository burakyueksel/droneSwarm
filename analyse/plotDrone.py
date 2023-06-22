import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import pdb # for debugging

# Read positions from the file
drone_data_all = {}
with open("drone_data.txt", "r") as file:
    for line in file:
        t, drone_id, x, y, z, qw, qx, qy, qz = map(float, line.strip().split())
        if drone_id not in drone_data_all:
            drone_data_all[drone_id] = []
        drone_data_all[drone_id].append([t, x, y, z, qw, qx, qy, qz])

# Create a 3D plot
fig0 = plt.figure(figsize=(8, 6))
ax0 = fig0.add_subplot(111, projection='3d')

# Plot every x-th point
every_xth_data = 100

# Plot the drone positions by ID
for drone_id, drone_data in drone_data_all.items():
    x = [data[1] for data in drone_data[::every_xth_data]]
    y = [data[2] for data in drone_data[::every_xth_data]]
    z = [-data[3] for data in drone_data[::every_xth_data]] # z comes in NED. Revert sign for negative down, positive up

    ax0.scatter(x, y, z, marker='.', label=f"Drone {int(drone_id)}", linewidths=0.1)

    # Plot the start and end positions as a black cross
    ax0.scatter(x[0], y[0], z[0], marker='x', color='black')
    ax0.scatter(x[-1], y[-1], z[-1], marker='x', color='black')

    # Compute heading direction from quaternion
    heading_directions = []
    for data in drone_data[::every_xth_data]:
        qw, qx, qy, qz = data[4:]
        roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        pitch = np.arcsin(2 * (qw * qy - qx * qz))
        heading = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        heading_directions.append(heading)

    # Plot the heading directions
    ax0.quiver(x, y, z, np.cos(heading_directions), np.sin(heading_directions), np.zeros_like(z),
              length=0.02, arrow_length_ratio=0.1, color='red', alpha=0.8)

# Set plot labels and title
ax0.set_xlabel('X [m]')
ax0.set_ylabel('Y [m]')
ax0.set_zlabel('Z [m]')
ax0.set_title('Drone Positions')
ax0.legend()



# Pllots on plane
fig = plt.figure(figsize=(8, 6))
# XY plot
ax_xy = fig.add_subplot(131)
for drone_id, drone_data in drone_data_all.items():
    x = [data[1] for data in drone_data[::every_xth_data]]
    y = [data[2] for data in drone_data[::every_xth_data]]
    ax_xy.plot(x, y, '.', label=f"Drone {int(drone_id)}")
    # Plot the start and end positions as a black cross
    ax_xy.plot(x[0], y[0], marker='x', color='black')
    ax_xy.plot(x[-1], y[-1], marker='x', color='black')

ax_xy.set_xlabel('X [m]')
ax_xy.set_ylabel('Y [m]')
ax_xy.set_title('XY Plane')
ax_xy.legend()

# XZ plot
ax_xz = fig.add_subplot(132)
for drone_id, drone_data in drone_data_all.items():
    x = [data[1] for data in drone_data[::every_xth_data]]
    z = [-data[3] for data in drone_data[::every_xth_data]]  # z comes in NED. Revert sign for negative down, positive up
    ax_xz.plot(x, z, '.', label=f"Drone {int(drone_id)}")
    # Plot the start and end positions as a black cross
    ax_xz.plot(x[0], z[0], marker='x', color='black')
    ax_xz.plot(x[-1], z[-1], marker='x', color='black')
ax_xz.set_xlabel('X [m]')
ax_xz.set_ylabel('Z [m]')
ax_xz.set_title('XZ Plane')
ax_xz.legend()

# YZ plot
ax_yz = fig.add_subplot(133)
for drone_id, drone_data in drone_data_all.items():
    y = [data[2] for data in drone_data[::every_xth_data]]
    z = [-data[3] for data in drone_data[::every_xth_data]]  # z comes in NED. Revert sign for negative down, positive up
    ax_yz.plot(y, z, '.', label=f"Drone {int(drone_id)}")
    # Plot the start and end positions as a black cross
    ax_yz.plot(y[0], z[0], marker='x', color='black')
    ax_yz.plot(y[-1], z[-1], marker='x', color='black')
ax_yz.set_xlabel('Y [m]')
ax_yz.set_ylabel('Z [m]')
ax_yz.set_title('YZ Plane')
ax_yz.legend()

# Create a separate figure for plotting z positions over time for each drone
fig2 = plt.figure(figsize=(8, 6))
ax2 = fig2.add_subplot(111)
# Iterate over each drone
for drone_id, drone_data in drone_data_all.items():
    # Extract z positions and time for the current drone
    time_plot = [data[0] for data in drone_data[::every_xth_data]]
    z_positions = [-data[3] for data in drone_data[::every_xth_data]]

    # Set a breakpoint to pause execution
    #pdb.set_trace()

    # Create a subplot for the current drone
    ax2.plot(time_plot, z_positions, label=f"Drone {int(drone_id)}")
    ax2.legend()
ax2.set_xlabel("Time")
ax2.set_ylabel("Z Position [m]")
ax2.set_title("Drone Z Position Over Time")
ax2.grid(True)


# Adjust the spacing between subplots
plt.tight_layout()

# Show the plot
plt.show()
