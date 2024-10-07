import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import calc_localizability

def csv_2_array(file_name):
    df = pd.read_csv(file_name)
    pointcloud = df[["x", "y", "z"]]
    eigenvalue = df[["eigen_value"]]

    array_points, array_values = np.array(pointcloud), np.array(eigenvalue)

    return array_points, array_values

file_name = "/home/rokuto/06_04_raw/00004708.csv"
num_x_divide, num_y_divide = 15, 15
array_points, array_values = csv_2_array(file_name)

x = array_points[:, 0]
y = array_points[:, 1]
z = array_points[:, 2]

x_range = np.max(x) - np.min(x)
y_range = np.max(y) - np.min(y)
x_step, y_step = x_range / num_x_divide, y_range / num_y_divide

table = np.hstack([array_points[:, 0:2], array_values])
points = np.zeros((num_x_divide, num_y_divide))
score = np.zeros((num_x_divide, num_y_divide)) 

#print("x_min:{} x_max:{}".format(np.min(x), np.max(x)))
#print("y_min:{} y_max:{}".format(np.min(y), np.max(y)))

for ix in range(num_x_divide):
    for iy in range(num_y_divide):
        left, right = np.min(x) + x_step * ix, np.min(x) + x_step * (ix + 1)
        under, upper = np.min(y) + y_step * iy, np.min(y) + y_step * (iy + 1)
        grid_in_points = table[((left <= table[:, 0]) & (table[:, 0] < right)) & ((under <= table[:, 1]) & (table[:, 1] < upper))]
        points[ix, iy] = grid_in_points.shape[0]
        score[ix, iy] = np.sum(grid_in_points[:, 2])

localizability = calc_localizability.global_score(score)
print(localizability)

sns.set()

fig, axes = plt.subplots(1, 2, figsize=(12, 6))

# 散布図を左側に描画
#sns.scatterplot(x=x, y=y, ax=axes[0])
axes[0].scatter(y, -x, c = array_values, cmap = "jet", s = 5)
axes[0].set_xlabel("x(m)")
axes[0].set_ylabel("y(m)")
axes[0].set_title("Scatter Plot")

"""
sns.heatmap(points, vmax = np.max(points), vmin = np.min(points), center=0, cmap='jet', annot=True, xticklabels=False, yticklabels=False, annot_kws={"size": 5}, fmt='.1f')
axes[1].set_title("Heatmap")
"""

sns.heatmap(score, vmax = np.max(score), vmin = np.min(score), center=0, cmap='jet', annot=True, xticklabels=False, yticklabels=False, annot_kws={"size": 5}, fmt='.1f')
axes[1].set_title("Score heatmap : localizability=:{}".format(round(localizability, 3)))

plt.show()

