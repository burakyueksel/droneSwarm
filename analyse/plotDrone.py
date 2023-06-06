import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Read positions from the file
positions = {}
with open("drone_positions.txt", "r") as file:
    for line in file:
        time, drone_id, x, y, z = map(float, line.strip().split())
        if drone_id not in positions:
            positions[drone_id] = []
        positions[drone_id].append([x, y, z])

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the drone positions by ID
for drone_id, drone_positions in positions.items():
    x = [position[0] for position in drone_positions]
    y = [position[1] for position in drone_positions]
    z = [-position[2] for position in drone_positions] # z comes in NED. Revert sign for negative down, positive up
    ax.scatter(x, y, z, marker='o', label=f"Drone {int(drone_id)}")

# Set plot labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Drone Positions')
ax.legend()

# Show the plot
plt.show()
