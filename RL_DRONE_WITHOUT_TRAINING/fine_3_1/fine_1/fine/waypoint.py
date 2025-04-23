import numpy as np
from scipy.special import comb
import random
import os
import matplotlib.pyplot as plt

def bernstein_poly(i, n, t):
    return comb(n, i) * (t ** i) * ((1 - t) ** (n - i))

def bezier_curve(control_points, num_points=1000):
    n = len(control_points) - 1
    t_values = np.linspace(0, 1, num_points)
    curve_points = np.zeros((num_points, 2))
    for i in range(n + 1):
        curve_points += bernstein_poly(i, n, t_values)[:, None] * control_points[i]
    return curve_points

def generate_control_points_monotonic(num_points=5):
    while True:
        pts = []
        pts.append([1, 1])  # Start at (1, 1) to ensure positive region

        intermediate = []
        for _ in range(num_points - 2):
            x = random.randint(1, 390)  # Ensure x is positive and within a reasonable range
            y = random.randint(1, 200)  # Ensure y is positive and within a reasonable range
            intermediate.append([x, y])
        
        # Sort intermediate points by x-coordinate to ensure monotonicity
        intermediate.sort(key=lambda point: point[0])
        pts.extend(intermediate)

        # Ensure end point is positive and at x = 400 to widen the path
        pts.append([400, random.randint(1, 200)])  # End y should also be positive

        pts = np.array(pts)
        # Check if the y values are in the desired range and the curve stays positive
        if np.all(pts[:, 1] > 0) and np.all(pts[:, 0] > 0):  # Make sure all points are positive
            return pts.tolist()

def save_bezier_points_to_file(curve_points, filename="waypoint.txt"):
    # Ensure the file is overwritten if it exists
    with open(filename, 'w') as f:
        for point in curve_points:
            # Save points as integers in the format: x y
            f.write(f"{int(point[0])} {int(point[1])}\n")

def plot_curve(control_points, curve_points):
    # Plot control points and Bezier curve
    control_points = np.array(control_points)
    curve_points = np.array(curve_points)

    plt.figure(figsize=(10, 6))
    
    # Plot Bezier curve
    plt.plot(curve_points[:, 0], curve_points[:, 1], label='Bezier Curve', color='blue')
    
    # Plot control points
    plt.scatter(control_points[:, 0], control_points[:, 1], color='red', label='Control Points', zorder=5)

    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Bezier Curve and Control Points')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    # Generate control points
    control_points = generate_control_points_monotonic(num_points=10)
    print("Control Points:", control_points)

    # Convert control points to numpy array for further processing
    control_points = np.array(control_points)
    
    # Generate Bezier curve points
    curve_points = bezier_curve(control_points)

    # Save the Bezier curve points to a file
    save_bezier_points_to_file(curve_points)

    # Plot the Bezier curve and control points
    plot_curve(control_points, curve_points)

if __name__ == "__main__":
    main()
