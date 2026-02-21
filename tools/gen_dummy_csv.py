import math
import random

def generate_spiral_csv(filename, num_points=10000):
    with open(filename, 'w') as f:
        f.write("x,y,z,intensity\n")
        for i in range(num_points):
            t = i / 100.0
            x = t * 0.1 * math.cos(t)
            y = t * 0.1 * math.sin(t)
            z = t * 0.05
            intensity = (math.sin(t) + 1) / 2
            f.write(f"{x:.4f},{y:.4f},{z:.4f},{intensity:.4f}\n")

if __name__ == "__main__":
    generate_spiral_csv("/Users/thomas/Desktop/points.csv")
    print("Generated synthetic /Users/thomas/Desktop/points.csv")

