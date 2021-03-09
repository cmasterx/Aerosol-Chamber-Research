import csv
import numpy as np
import matplotlib.pyplot as plt
from numpy.lib.function_base import diff

file = open('data.csv', 'r')
csvfile = csv.reader(file)

data = []
for row in csvfile:
    data.append(row)

data = np.array(data)
data = np.delete(data, -1, 1)
data = np.float32(data)

diff0 = data[:,1] - data[:,4]
diff1 = data[:,1] - data[:,7]

diff0 = diff0 - diff0[0]
diff1 = diff1 - diff1[0]

# plt.plot(diff0)
# plt.plot(diff1)

ft = np.fft.fft(diff0)
low_pass = [(i / (1 + (i / (2 * np.pi * 20)) ** 2)**0.5) for i in range(len(ft))]

low_sig = ft * low_pass
low_sig = np.fft.ifft(low_sig)


print(np.std(diff0))
print(np.std(diff1))
plt.plot(data[:,1])
plt.plot(data[:,4])
plt.plot(data[:,7])
plt.figure()
print(ft.shape)
# plt.plot(ft)
# plt.plot(low_sig)
plt.plot(diff0)
plt.plot(diff1)
# plt.figure()
plt.show()
print(data[:,-1])
