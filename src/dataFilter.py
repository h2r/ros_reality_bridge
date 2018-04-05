#!/usr/bin/env python
from scipy import optimize
import numpy as np
import matplotlib.pyplot as plt

class DataFilter(object):
	"""docstring for ClassName"""
	def __init__(self):
		pass

	def filter(self, data, supports):
		num_points = len(data)
		x = data[:,0]
		y = data[:,1]
		z = data[:,2]
		t = np.arange(num_points)
		sigma = np.ones(num_points)
		if len(supports) > 0:
			sigma[[supports]] = 0.01
		# p1, _ = optimize.curve_fit(self.f, x, y, (0,) * (num_points + 1) , sigma=sigma)
		# p2, _ = optimize.curve_fit(self.f, x, y, (0,) * (num_points + 1))
		# plt.plot(x, y, "o")
		degrees = min((len(supports) + 1), min(num_points, 10))
		guesses = (0, ) * degrees
		x_fit, _ = optimize.curve_fit(self.f, t, x, guesses, sigma=sigma)
		y_fit, _ = optimize.curve_fit(self.f, t, y, guesses, sigma=sigma)
		z_fit, _ = optimize.curve_fit(self.f, t, z, guesses, sigma=sigma)
		# p2, _ = optimize.curve_fit(self.f, x, y)

		# plt.figure(0)
		# plt.plot(t, x, "o")
		# plt.plot(t, self.f(t, *x_fit), "r", label=u"x data")
		# plt.show()

		# plt.figure(1)
		# plt.plot(t, y, "o")
		# plt.plot(t, self.f(t, *y_fit), "b", label=u"y data")
		# plt.show()

		# plt.figure(2)
		# plt.plot(t, z, "o")
		# plt.plot(t, self.f(t, *z_fit), "g", label=u"z data")
		# plt.show()
		return np.array([self.f(t, *x_fit), self.f(t, *y_fit), self.f(t, *z_fit)]).T.tolist()

	def f(self, t, *p):
		return np.poly1d(p)(t)



# points = np.array([[1.0, 3.0, 5.0], [1.2, 2.9, 5.2], [1.3, 2.8, 5.3], [1.4, 2.7, 5.4], [1.25, 2.76, 5.2], [1.0, 3.0, 5.0], [1.0, 2.93, 4.8], [1.5, 3.2, 4.8], [1.6, 3.3, 4.6]])
# supports = np.array([0, 1, 3, 4, 8])
# print DataFilter().filter(points, supports)

