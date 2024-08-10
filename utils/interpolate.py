from scipy import interpolate
import numpy as np

if __name__ == "__main__":
    data = np.genfromtxt("/home/joe/Desktop/position_pose_velocity.txt", dtype=float)
    data = data[:, 0:3]
    
    