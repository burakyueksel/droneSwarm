import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import numpy as np

# Initialize the figure and axes for 3D plot
fig = plt.figure()
ax_3d = fig.add_subplot(121, projection='3d')

# Initialize the figure and axes for xz-plane projection
ax_xz = fig.add_subplot(122)
ax_xz.set_xlabel('X')
ax_xz.set_ylabel('Z')

# Read the data from the file
data = np.loadtxt('drone_positions.txt', delimiter=' ', skiprows=1)
current_times = data[:, 0]
drone_ids = data[:, 1]
positions = data[:, 2:]

# Keep track of the current frame
current_frame = 0

# Define the update function for the animation
def update(frame):
    global current_frame

    # Clear the 3D plot
    ax_3d.cla()

    # Plot the positions up to the current frame in 3D
    for drone_id in np.unique(drone_ids):
        mask = np.logical_and(drone_ids == drone_id, np.arange(len(drone_ids)) <= current_frame)
        ax_3d.plot(positions[mask, 0], positions[mask, 1], positions[mask, 2], label=f'Drone {int(drone_id)}')

    # Set plot labels and legend for 3D plot
    ax_3d.set_xlabel('X')
    ax_3d.set_ylabel('Y')
    ax_3d.set_zlabel('Z')
    ax_3d.legend()

    # Set the z-axis limits for 3D plot
    ax_3d.set_zlim(-50, 50)

    # Clear the xz-plane projection plot
    ax_xz.cla()

    # Plot the positions up to the current frame in xz-plane projection
    for drone_id in np.unique(drone_ids):
        mask = np.logical_and(drone_ids == drone_id, np.arange(len(drone_ids)) <= current_frame)
        ax_xz.plot(positions[mask, 0], positions[mask, 2], label=f'Drone {int(drone_id)}')

    # Set plot labels and legend for xz-plane projection
    ax_xz.set_xlabel('X')
    ax_xz.set_ylabel('Z')
    ax_xz.legend()

    # Increment the current frame
    current_frame += 1000

# Create the animation
ani = FuncAnimation(fig, update, frames=len(positions), interval=1, blit=False)

# Save the animation as a GIF file
#ani.save('animation.gif', writer='pillow')

# Show the plot
plt.show()
