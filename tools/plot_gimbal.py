#!/usr/bin/python

import numpy
import pylab
import sys

lines = open(sys.argv[1]).readlines()
data = [tuple(float(x) for x in line.split()) for line in lines]
data = sorted([(y, float(int(x) % 256)) for x, y in data])
print data

x = numpy.array([a[0] for a in data])
y = numpy.array([a[1] for a in data])

offset = 0
old = y[0]
for i in range(len(y)):
    if abs(y[i] - old) > 128:
        if y[i] > old:
            offset -= 256
        else:
            offset += 256
    old = y[i]
    y[i] += offset

print x
print y


fit = numpy.polyfit(x, y, deg=1)

print "fit:", fit
print 1/fit[0]
print fit[1]/fit[0]

pylab.subplot(211)
pylab.grid()

pylab.plot(x, fit[0] * x + fit[1], color='red')
pylab.plot(x, y)

pylab.subplot(212)
pylab.grid()
pylab.plot(x, fit[0] * x + fit[1] - y, '+')

pylab.show()
