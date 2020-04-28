#! /usr/bin/env python

import sys

bpm = float(sys.argv[1])
IE = float(sys.argv[2])
th = float(sys.argv[3])
print('bpm=%f, IE=%f, th=%f' % (bpm, IE, th))

tb = 60.0/bpm
ti = tb/(1+IE) - th
if ti <= 0:
    print('bad parameter combination, insp time <= 0')
te = IE * (ti+th)

print('ti=%f, te=%f, IE=%f, tot=%f' % (ti, te, te/(ti+th), ti+te+th))