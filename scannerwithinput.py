import numpy as np
import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import ConvexHull
from stl import mesh
import time

# Set up Serial connection
SERIAL_PORT = "COM5"  # Change this to match your Arduino port (e.g., /dev/ttyUSB0 for Linux)
BAUD_RATE = 115200

def polar_to_cartesian(distance, angle, height):
    """
    Converts polar coordinates (distance, angle, height) to Cartesian (X, Y, Z).
    """
    x = distance * np.cos(angle)
    y = distance * np.sin(angle)
    z = height
    return [x, y, z]

def read_serial_data():
    """
    Reads and processes scanner data from the serial port.
    """
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Allow Arduino to reset
    points = []
    last_line = None  # Store the last read line

    print("Reading data from Arduino... Press Ctrl+C to stop.")

    try:
        while True:
            line = ser.readline().decode("utf-8").strip()
            if line and line != last_line:  # Ignore repeated lines
                last_line = line  # Store the last read line
                try:
                    h, a, d = map(float, line.split(","))
                    a = np.radians(a)  # Convert degrees to radians
                    points.append(polar_to_cartesian(d, a*0.3515625, h*0.37))
                    print(d,a*0.3515625,h*3.7)
                except ValueError:
                    print(f"Invalid data received: {line}")
    except KeyboardInterrupt:
        print("\nData collection stopped.")
        ser.close()

    return np.array(points)


def save_as_stl(points, filename="scanned_model.stl"):
    """
    Saves the 3D scanned model as an STL file.
    """
    #points = np.unique(points, axis=0)  # Remove duplicates

    if len(points) < 4:
        print("Error: Not enough unique points for a 3D Convex Hull!")
        return

    try:
        hull = ConvexHull(points)
        faces = np.array([points[simplex] for simplex in hull.simplices])

        model_mesh = mesh.Mesh(np.zeros(len(faces), dtype=mesh.Mesh.dtype))
        for i, f in enumerate(faces):
            model_mesh.vectors[i] = f

        model_mesh.save(filename)
        print(f"✅ 3D model saved as {filename}")

    except Exception as e:
        print(f"❌ Convex Hull computation failed: {e}")

def plot_3d(points):
    """
    Plots the 3D model ensuring only outer edges are visible and faces are fully filled.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    if len(points) >= 4:  # Convex hull requires at least 4 points
        try:
            hull = ConvexHull(points)
            faces = [points[simplex] for simplex in hull.simplices]
            
            ax.add_collection3d(Poly3DCollection(faces, alpha=0.9, facecolor='cyan', edgecolor='black', linewidth=1))
        
        except Exception as e:
            print(f"Convex Hull error while plotting: {e}")

    # Auto-scale plot
    x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
    y_min, y_max = np.min(points[:, 1]), np.max(points[:, 1])
    z_min, z_max = np.min(points[:, 2]), np.max(points[:, 2])
    
    padding = 0.1
    ax.set_xlim([x_min - padding, x_max + padding])
    ax.set_ylim([y_min - padding, y_max + padding])
    ax.set_zlim([z_min - padding, z_max + padding])
    
    z_values = points[:, 2]
    top_index = np.argmax(z_values)
    bottom_index = np.argmin(z_values)

    top_center = np.mean(points[:, :2], axis=0)
    ax.text(top_center[0], top_center[1], points[top_index, 2] + 3, 'Top', color='red', fontsize=12, fontweight='bold', ha='center')
    bottom_center = np.mean(points[:, :2], axis=0)
    ax.text(bottom_center[0], bottom_center[1], points[bottom_index, 2] - 3, 'Bottom', color='blue', fontsize=12, fontweight='bold', ha='center')
    
    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")
    plt.title("3D Scanned Model (Fully Filled Shape)")
    plt.show()

# === RUN THE PROCESS ===
cartesian_points = read_serial_data()
if len(cartesian_points) > 0:
    plot_3d(cartesian_points)
    save_as_stl(cartesian_points)
else:
    print("No valid data received!")