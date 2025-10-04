import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation

PORT = 'COM8'
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Connected to {PORT}")
except Exception as e:
    print(f"Error opening serial port: {e}")
    ser = None

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_title("3D Roll & Pitch Visualization (MPU6050)")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

# Simple box vertices
vertices = np.array([
    [-0.5, -0.3, -0.1],
    [ 0.5, -0.3, -0.1],
    [ 0.5,  0.3, -0.1],
    [-0.5,  0.3, -0.1],
    [-0.5, -0.3,  0.1],
    [ 0.5, -0.3,  0.1],
    [ 0.5,  0.3,  0.1],
    [-0.5,  0.3,  0.1]
])
faces = [
    [0,1,2,3], [4,5,6,7],
    [0,1,5,4], [2,3,7,6],
    [0,3,7,4], [1,2,6,5]
]

def rotation_matrix(pitch, roll):
    """Rotation from pitch (Y-axis) and roll (X-axis)"""
    pitch = np.radians(pitch)
    roll = np.radians(roll)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    return Ry @ Rx

def update(frame):
    pitch = roll = 0
    if ser and ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            parts = line.split(',')
            if len(parts) >= 2:
                pitch = float(parts[0])
                roll = float(parts[1])
        except:
            pass

    ax.cla()
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(f"3D Roll & Pitch | Pitch: {pitch:.1f}°, Roll: {roll:.1f}°")

    R = rotation_matrix(pitch, roll)
    rotated = vertices @ R.T
    for f in faces:
        face = [rotated[i] for i in f]
        poly = Poly3DCollection([face], color='skyblue', edgecolor='k', alpha=0.9)
        ax.add_collection3d(poly)
    return []

ani = FuncAnimation(fig, update, interval=50)
plt.show()

if ser and ser.is_open:
    ser.close()
