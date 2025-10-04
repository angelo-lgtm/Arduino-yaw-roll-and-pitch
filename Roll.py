import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


PORT = 'COM8'         
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Connected to {PORT}")
except Exception as e:
    print(f"Error opening serial port: {e}")
    ser = None

fig, ax = plt.subplots()
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_aspect('equal')
ax.set_title("2D Roll Visualization (MPU6050)")
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")

# Draw base car rectangle
car_width = 1.0
car_height = 0.5
car_rect, = ax.plot([], [], color='blue', lw=3)

def rotate_rectangle(roll_deg):
    """Return coordinates of a rotated 2D rectangle based on roll angle"""
    roll = np.radians(roll_deg)
    # rectangle centered at origin
    corners = np.array([
        [-car_width/2, -car_height/2],
        [ car_width/2, -car_height/2],
        [ car_width/2,  car_height/2],
        [-car_width/2,  car_height/2],
        [-car_width/2, -car_height/2]
    ])
    R = np.array([[np.cos(roll), -np.sin(roll)],
                  [np.sin(roll),  np.cos(roll)]])
    rotated = corners @ R.T
    return rotated[:,0], rotated[:,1]

def update(frame):
    roll = 0.0
    if ser and ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            parts = line.split(',')
            if len(parts) >= 2:
                roll = float(parts[1])  # second value is roll
        except:
            pass

    x, y = rotate_rectangle(roll)
    car_rect.set_data(x, y)
    ax.set_title(f"2D Roll Visualization | Roll: {roll:.1f}°")
    return car_rect,

ani = FuncAnimation(fig, update, interval=50, blit=True)
plt.show()

if ser and ser.is_open:
    ser.close()
