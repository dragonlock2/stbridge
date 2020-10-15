import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import stbridge

ADDR = 0x6a

stbridge.open()
stbridge.initI2C(400)

stbridge.writeI2C(ADDR, [0x10, 0b10100000]) # accel settings
stbridge.writeI2C(ADDR, [0x11, 0b10101000]) # gyro settings

# pybridge.writeI2C(ADDR, [0x28]) # accel
# a = pybridge.readI2C(ADDR, 6)

# pybridge.writeI2C(ADDR, [0x22]) # gyro
# g = pybridge.readI2C(ADDR, 6)

# graphing stuff
MAX_POINTS = 500
INTERVAL = 15 #ms

xs = list(range(MAX_POINTS))
yss = [[0]*MAX_POINTS for i in range(6)]

fig = plt.figure(figsize=(12, 5))
ax = plt.axes(xlim=(0, MAX_POINTS-1), ylim=(-32768, 32767))
lines = []
lines.append(ax.plot(xs, yss[0], label='x_accel')[0])
lines.append(ax.plot(xs, yss[1], label='y_accel')[0])
lines.append(ax.plot(xs, yss[2], label='z_accel')[0])
lines.append(ax.plot(xs, yss[3], label='x_gyro')[0])
lines.append(ax.plot(xs, yss[4], label='y_gyro')[0])
lines.append(ax.plot(xs, yss[5], label='z_gyro')[0])
ax.legend(bbox_to_anchor=(1,1), loc='upper left')

def update(i):
	# get data
	stbridge.writeI2C(ADDR, [0x28]) # accel
	dat = struct.unpack('<3h', bytes(stbridge.readI2C(ADDR, 6)))
	stbridge.writeI2C(ADDR, [0x22]) # gyro
	dat += struct.unpack('<3h', bytes(stbridge.readI2C(ADDR, 6)))

	# update our lists
	for val, ys in zip(dat, yss):
		ys.append(val)
		ys.pop(0)

	# plot the data
	for i in range(6):
		lines[i].set_ydata(yss[i])

	return lines

print('Close graph to exit...')
ani = animation.FuncAnimation(fig, update, interval=INTERVAL, blit=True)
plt.show()

stbridge.close()