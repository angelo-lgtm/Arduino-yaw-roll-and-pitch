import utils.session
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ---------------- CONFIG ----------------
PORT = 'COM8'          # your Arduino port
BAUD = 115200
WINDOW = 200           # number of samples to display

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Connected to {PORT}")
except Exception as e:
    print(f"Error opening serial port: {e}")
    ser = None

# Initialize with default values
pitch, roll, yaw = 0.0, 0.0, 0.0
pitch_buf = deque(maxlen=WINDOW)
roll_buf  = deque(maxlen=WINDOW)
yaw_buf   = deque(maxlen=WINDOW)
x_idx     = deque(maxlen=WINDOW)

# Car state
car_x, car_y = 0.0, 0.0
car_speed = 0.0

# ---------------- FIGURE ----------------
fig = plt.figure(figsize=(14, 8))

# Time-series subplot
ax1 = fig.add_subplot(2, 1, 1)
line_pitch, = ax1.plot([], [], label="Pitch", color='red', linewidth=2)
line_roll,  = ax1.plot([], [], label="Roll", color='blue', linewidth=2)
line_yaw,   = ax1.plot([], [], label="Yaw", color='green', linewidth=2)
ax1.set_xlim(0, WINDOW)
ax1.set_ylim(-180, 180)
ax1.set_xlabel("Samples")
ax1.set_ylabel("Angle (°)")
ax1.set_title("MPU6050 Pitch, Roll & Yaw - Real-time Data")
ax1.legend(loc='upper right')
ax1.grid(True, alpha=0.3)

# 3D Car subplot
ax2 = fig.add_subplot(2, 1, 2, projection='3d')
ax2.set_xlim([-2, 2])
ax2.set_ylim([-1.5, 1.5])
ax2.set_zlim([0, 1])
ax2.set_title("3D Car - Controlled by MPU6050")
ax2.set_xlabel("X")
ax2.set_ylabel("Y")
ax2.set_zlabel("Z")

# SIMPLE CAR MODEL - Using basic shapes that will definitely display
def create_simple_car():
    # Simple box car - fewer vertices, simpler structure
    vertices = [
        # Car body - bottom rectangle (z=0)
        [-1.0, -0.4, 0], [1.0, -0.4, 0],  # front-left, front-right bottom
        [1.0, 0.4, 0], [-1.0, 0.4, 0],    # rear-right, rear-left bottom
        
        # Car body - top rectangle (z=0.5)  
        [-1.0, -0.4, 0.5], [1.0, -0.4, 0.5],  # front-left, front-right top
        [1.0, 0.4, 0.5], [-1.0, 0.4, 0.5],    # rear-right, rear-left top
        
        # Roof - smaller rectangle on top (z=0.5 to z=0.8)
        [-0.6, -0.3, 0.5], [0.6, -0.3, 0.5],  # front-left, front-right roof bottom
        [0.6, 0.3, 0.5], [-0.6, 0.3, 0.5],    # rear-right, rear-left roof bottom
        [-0.6, -0.3, 0.8], [0.6, -0.3, 0.8],  # front-left, front-right roof top
        [0.6, 0.3, 0.8], [-0.6, 0.3, 0.8],    # rear-right, rear-left roof top
        
        # Wheels - simple rectangles
        # Front left wheel
        [-0.7, -0.5, 0.1], [-0.5, -0.5, 0.1], [-0.5, -0.5, 0.3], [-0.7, -0.5, 0.3],
        [-0.7, -0.4, 0.1], [-0.5, -0.4, 0.1], [-0.5, -0.4, 0.3], [-0.7, -0.4, 0.3],
        
        # Front right wheel
        [-0.7, 0.4, 0.1], [-0.5, 0.4, 0.1], [-0.5, 0.4, 0.3], [-0.7, 0.4, 0.3],
        [-0.7, 0.5, 0.1], [-0.5, 0.5, 0.1], [-0.5, 0.5, 0.3], [-0.7, 0.5, 0.3],
        
        # Rear left wheel
        [0.5, -0.5, 0.1], [0.7, -0.5, 0.1], [0.7, -0.5, 0.3], [0.5, -0.5, 0.3],
        [0.5, -0.4, 0.1], [0.7, -0.4, 0.1], [0.7, -0.4, 0.3], [0.5, -0.4, 0.3],
        
        # Rear right wheel
        [0.5, 0.4, 0.1], [0.7, 0.4, 0.1], [0.7, 0.4, 0.3], [0.5, 0.4, 0.3],
        [0.5, 0.5, 0.1], [0.7, 0.5, 0.1], [0.7, 0.5, 0.3], [0.5, 0.5, 0.3],
    ]
    return np.array(vertices)

# Define faces for the simple car - using explicit vertex indices
car_faces = [
    # Main body - bottom and top
    [0, 1, 2, 3],      # bottom face
    [4, 5, 6, 7],      # top face
    [0, 1, 5, 4],      # front face
    [2, 3, 7, 6],      # back face
    [0, 3, 7, 4],      # left side
    [1, 2, 6, 5],      # right side
    
    # Roof bottom and top
    [8, 9, 10, 11],    # roof bottom
    [12, 13, 14, 15],  # roof top
    [8, 9, 13, 12],    # roof front
    [10, 11, 15, 14],  # roof back
    [8, 11, 15, 12],   # roof left
    [9, 10, 14, 13],   # roof right
    
    # Connect body to roof
    [4, 5, 9, 8],      # front connection
    [6, 7, 11, 10],    # back connection
    
    # Front left wheel - 6 faces for a box
    [16, 17, 18, 19], [20, 21, 22, 23],
    [16, 17, 21, 20], [18, 19, 23, 22],
    [16, 19, 23, 20], [17, 18, 22, 21],
    
    # Front right wheel
    [24, 25, 26, 27], [28, 29, 30, 31],
    [24, 25, 29, 28], [26, 27, 31, 30],
    [24, 27, 31, 28], [25, 26, 30, 29],
    
    # Rear left wheel
    [32, 33, 34, 35], [36, 37, 38, 39],
    [32, 33, 37, 36], [34, 35, 39, 38],
    [32, 35, 39, 36], [33, 34, 38, 37],
    
    # Rear right wheel
    [40, 41, 42, 43], [44, 45, 46, 47],
    [40, 41, 45, 44], [42, 43, 47, 46],
    [40, 43, 47, 44], [41, 42, 46, 45],
]

# Colors for each face
car_colors = [
    'darkblue', 'darkblue', 'blue', 'blue', 'blue', 'blue',  # main body
    'red', 'red', 'red', 'red', 'red', 'red',  # roof
    'gray', 'gray',  # connections
    'black', 'black', 'black', 'black', 'black', 'black',  # front left wheel
    'black', 'black', 'black', 'black', 'black', 'black',  # front right wheel
    'black', 'black', 'black', 'black', 'black', 'black',  # rear left wheel
    'black', 'black', 'black', 'black', 'black', 'black',  # rear right wheel
]

car_vertices = create_simple_car()

def rotation_matrix(yaw, pitch, roll):
    """Create rotation matrix from yaw, pitch, roll angles"""
    yaw_rad = np.radians(yaw)
    pitch_rad = np.radians(pitch)
    roll_rad = np.radians(roll)

    # Rotation around Z axis (yaw)
    Rz = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
        [np.sin(yaw_rad), np.cos(yaw_rad), 0],
        [0, 0, 1]
    ])
    
    # Rotation around Y axis (pitch)
    Ry = np.array([
        [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
        [0, 1, 0],
        [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
    ])
    
    # Rotation around X axis (roll)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll_rad), -np.sin(roll_rad)],
        [0, np.sin(roll_rad), np.cos(roll_rad)]
    ])
    
    # Combined rotation matrix
    return Rz @ Ry @ Rx

def update_car_movement(pitch, roll, yaw):
    """Update car position based on MPU6050 data"""
    global car_x, car_y, car_speed
    
    # Use pitch to control forward/backward movement
    car_speed = -pitch * 0.01  # Negative so forward tilt moves forward
    
    # Use roll to control left/right movement
    steering = roll * 0.01
    
    # Update position
    car_x += car_speed
    car_y += steering
    
    # Keep car within bounds
    car_x = max(-1.5, min(1.5, car_x))
    car_y = max(-1.0, min(1.0, car_y))
    
    return car_x, car_y

def update(frame):
    global pitch, roll, yaw
    
    # Read data from Arduino
    if ser and ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                parts = line.split(',')
                if len(parts) == 3:
                    pitch = float(parts[0])
                    roll = float(parts[1]) 
                    yaw = float(parts[2])
                    
                    # Store data for plotting
                    pitch_buf.append(pitch)
                    roll_buf.append(roll)
                    yaw_buf.append(yaw)
                    x_idx.append(len(x_idx) + 1)
                    
                    print(f"MPU Data - Pitch: {pitch:.1f}°, Roll: {roll:.1f}°, Yaw: {yaw:.1f}°")
                    
        except Exception as e:
            print(f"Error reading serial: {e}")

    # Update time-series plot
    xs = list(range(len(x_idx)))
    if xs:
        line_pitch.set_data(xs, list(pitch_buf))
        line_roll.set_data(xs, list(roll_buf))
        line_yaw.set_data(xs, list(yaw_buf))
        ax1.set_xlim(max(0, len(xs) - WINDOW), max(WINDOW, len(xs)))

    # Update car position based on MPU data
    car_x, car_y = update_car_movement(pitch, roll, yaw)
    
    # Clear and update 3D plot
    ax2.clear()
    ax2.set_xlim([-2, 2])
    ax2.set_ylim([-1.5, 1.5])
    ax2.set_zlim([0, 1])
    ax2.set_title(f"3D Car - MPU Controlled | Pos: ({car_x:.2f}, {car_y:.2f})")
    ax2.set_xlabel("X")
    ax2.set_ylabel("Y")
    ax2.set_zlabel("Z")
    
    # Create rotation matrix from MPU angles
    R = rotation_matrix(yaw, pitch, roll)
    
    # Apply rotation to car vertices
    rotated_vertices = (car_vertices @ R.T)
    
    # Apply position translation
    positioned_vertices = rotated_vertices + np.array([car_x, car_y, 0])
    
    # Draw the car
    for i, face in enumerate(car_faces):
        if i < len(car_colors):
            color = car_colors[i]
        else:
            color = 'gray'
        
        try:
            # Get vertices for this face
            face_vertices = [positioned_vertices[vertex_idx] for vertex_idx in face]
            
            # Create polygon
            poly = Poly3DCollection([face_vertices], 
                                  facecolors=color, 
                                  edgecolors='black',
                                  alpha=0.8,
                                  linewidths=1)
            ax2.add_collection3d(poly)
        except Exception as e:
            print(f"Error drawing face {i}: {e}")
            continue

    return line_pitch, line_roll, line_yaw

print("Starting MPU6050 Car Control Visualization...")
print("Control the car by tilting the MPU6050:")
print("- Tilt FORWARD/BACKWARD to move car along X-axis")
print("- Tilt LEFT/RIGHT to move car along Y-axis") 
print("- ROTATE to change car orientation")
print("Press Ctrl+C to stop")

try:
    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()
except KeyboardInterrupt:
    print("\nStopping visualization...")
finally:
    if ser and ser.is_open:
        ser.close()
        print("Serial port closed")