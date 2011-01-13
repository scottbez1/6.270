#!/usr/bin/env python
import sys

# usage e.g.
# ./tag.py 2048; /Applications/Inkscape.app/Contents/Resources/bin/inkscape -f out.svg -A out.pdf; cp out.pdf ~/Sites/tag.pdf

n = int(sys.argv[1])

use16 = 1
if use16:
    bits = [(n >> i) % 2 for i in range(0,16)]
else:
    bits = [(n >> i) % 2 for i in range(0,12)]

    # 1fg0
    # habi
    # jcdk
    # elmn
    bits = \
    [1] + bits[4:6] + [0] + \
    bits[6:7] + bits[0:2] + bits[7:8] + \
    bits[8:9] + bits[2:4] + bits[9:10] + \
    [0] + bits[10:12] + [0]

    extra = (n >> 12) % 3
    if extra > 0:
        bits[12] = 1
    if extra > 1:
        bits[15] = 1
print bits

a = [aa+'\n' for aa in open('template.svg','r').read().split('\n\n')]

f = open('out.svg','w')

inch = 90
blackborder = 1.0 * inch
whiteborder = .5 * inch
square = 1.0 * inch
white = '#ffffff'
black = '#000000'
id = 0
rows = 4
cols = 4
origin = whiteborder+blackborder

totalwidth = origin*2 + cols*square
totalheight = origin*2 + rows*square

f.write(a[0] % {'width':totalwidth, 'height':totalheight})

def svgRect(**kwargs):
    global id
    style = {} if 'style' not in kwargs else kwargs['style']
    if 'fill' in kwargs:
        del kwargs['fill']
    if 'stroke' in kwargs:
        style['stroke'] = kwargs['stroke']
        del kwargs['stroke']
    kwargs['style'] = ';'.join(['%s:%s'%(k,v) for k,v in style.iteritems()])
    kwargs['id'] = 'r%d' % id
    tag = '  <rect ' + ' '.join(['%s="%s"'%(k,v) for k,v in kwargs.iteritems()]) + ' />\n'
    id += 1
    f.write(tag)

svgRect(x=0, y=0, width=totalwidth, height=totalheight, style=dict(fill=black))
svgRect(x=blackborder, y=blackborder, width=whiteborder*2+cols*square,
        height=whiteborder*2+rows*square, style=dict(fill=white))

for i in range(rows):
    for j in range(cols):
        if not bits[i*rows+j]:
            svgRect(x=origin+j*square, y=origin+i*square, width=square,
                    height=square, style=dict(fill=black, stroke=black,
                    **{'stroke-width':'.5pt'}))

f.write(a[1])
f.close()
