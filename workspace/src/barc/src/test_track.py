import numpy as np
import matplotlib.pyplot as plt

def rotate_around_z(xy_coords, angle):
    rotation_matrix = np.array([[np.cos(angle), - np.sin(angle)],
                                [np.sin(angle), np.cos(angle)]])

    return np.dot(rotation_matrix, xy_coords)

alpha = np.arange(0.0, 2 * np.pi, 0.005)
r = 1.0

init_point = np.array([0.0, r])
plt.plot(init_point[0], init_point[1], "bo")

starting = np.array([0.0, 0.0])
prev_angle = 0.0

num_points = 18

xy = np.zeros((num_points + 1, 2))

prev_angle = 0.0

for i in range(1, num_points + 1):
	print(i)
	print("angle before: ", - (i * 10.0) / 180.0 * np.pi)
	print("prev angle: ", prev_angle)
	angle = - (i * 10.0) / 180.0 * np.pi - prev_angle
	l = - 2 * r * np.sin(angle / 2)
	print("angle: ", angle)

	starting[0] = l

	plt.plot(starting[0], starting[1], "co")
	print(starting)

	delta_xy = rotate_around_z(starting, angle / 2)

	plt.plot(delta_xy[0], delta_xy[1], "go")

	xy[i, :] = delta_xy # + xy[i - 1, :]

	prev_angle = angle

# xy = np.cumsum(xy, 0)
xy += init_point

plt.plot(xy[:, 0], xy[:, 1], "ko")

plt.plot(r * np.cos(alpha), r * np.sin(alpha), "r-")

# plt.plot(init_point[0], init_point[1], "bo")


plt.xlim([- r - 0.2, r + 0.2])
plt.ylim([- r - 0.2, r + 0.2])
plt.axis("equal")
plt.show()
