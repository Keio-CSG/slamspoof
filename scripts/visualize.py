#!/usr/bin/env python3

import numpy as np
import pandas as pd
import rospy
import matplotlib.pyplot as plt

def visualizer_main():
    smvs_file = rospy.get_param('smvs_file_name', '/home/') 
    df = pd.read_csv(smvs_file)
    df_x, df_y, df_smvs = df['x'], df['y'], df['smvs']

    points = plt.scatter(np.array(df_x), np.array(df_y), c=np.array(df_smvs), cmap='jet')
    cbar = plt.colorbar(points)
    cbar.set_label("SMVS")

    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.show()

if __name__ == "__main__":
    visualizer_main()