#!/usr/bin/env python
import sys

n = int(sys.argv[1])

bits = [(n >> i) % 2 for i in range(0,16-4)]
# 1ef0
# gabh
# icdj
# 0kl0
bits = \
[1] + bits[4:6] + [0] + \
bits[6:7] + bits[0:2] + bits[7:8] + \
bits[8:9] + bits[2:4] + bits[9:10] + \
[0] + bits[10:12] + [0]
print bits

a = [aa+'\n' for aa in open('template.svg','r').read().split('\n\n')]

f = open('out.svg','w')

inch = 90
blackborder = .5 * inch
whiteborder = .3 * inch
square = 1.0 * inch
white = 'ffffff'
black = '000000'
id = 0
rows = 4
cols = 4
origin = whiteborder+blackborder

totalwidth = origin*2 + cols*square
totalheight = origin*2 + rows*square

f.write(a[0] % {'width':totalwidth, 'height':totalheight})

f.write(a[1] % {'x':0, 'y':0,
                'width':totalwidth,
                'height':totalheight,
                'id':id, 'color':black}); id+=1

f.write(a[1] % {'x':blackborder, 'y':blackborder,
                'width':(whiteborder*2+cols*square),
                'height':(whiteborder*2+rows*square),
                'id':id, 'color':white}); id+=1

for i in range(rows):
    for j in range(cols):
        f.write(a[1] % {'x':origin+j*square, 'y':origin+i*square,
                        'width':square, 'height':square,
                        'id':id, 'color':white if bits[i*rows+j] else black}); id+=1

f.write(a[2])
f.close()
