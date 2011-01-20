#!/usr/bin/env python
from pylab import *
from numpy import *

POPCOUNT_TABLE16 = [0] * 2**16
for index in xrange(len(POPCOUNT_TABLE16)):
    POPCOUNT_TABLE16[index] = (index & 1) + POPCOUNT_TABLE16[index >> 1]

def popcount(v):
    return POPCOUNT_TABLE16[v & 0xffff]

def bin(s):
    return str(s) if s<=1 else bin(s>>1) + str(s&1)

def rot(n):
    bits = ''.join([str(d) for d in array([int(d) for d in ('%16s' % bin(n)).replace(' ','0')]).reshape(4,4).T[:,::-1].reshape(16)])
    return int(bits,2)

def makeSquare(pt):
    return array([int(d) for d in ('%16s' % bin(pt)).replace(' ','0')]).reshape(4,4)[::-1,::-1]

rot = vectorize(rot)

use16 = 1
if use16:
    def hamm(a, b):
        return popcount(a^b)
    N=1<<16
else:
    def hamm(a, b):
        # left out bit 0
        a = ((a&0x30)>>3) | ((a&0x40)>>2) | ((a&0x03)<<5) | (a&0x180) | ((a&0x0c)<<7) | ((a&0x200)<<2) | (((a>>12)>0)<<12) | ((a&0xc00)<<3) | (((a>>12)>1)<<15)
        b = ((b&0x30)>>3) | ((b&0x40)>>2) | ((b&0x03)<<5) | (b&0x180) | ((b&0x0c)<<7) | ((b&0x200)<<2) | (((b>>12)>0)<<12) | ((b&0xc00)<<3) | (((b>>12)>1)<<15)
        return popcount(a^b)
    N = (1<<12)*3

hamm = vectorize(hamm)

r = arange(1<<16)
rr = rot(r)
rr = vstack((r,rr,rr[rr],rr[rr[rr]]))

r = arange(N)

if use16:
    good = all(rr[1:]>r[newaxis,:],axis=0)
    r = r[where(good)]
else:
    r = arange(N)
    r = 1 + ((r&0x30)>>3) | ((r&0x40)>>2) | ((r&0x03)<<5) | (r&0x180) | ((r&0x0c)<<7) | ((r&0x200)<<2) | (((r>>12)>0)<<12) | ((r&0xc00)<<3) | (((r>>12)>1)<<15)

random.seed(1338)
random.shuffle(r)

def attempt(n=None,M=32,visual=False):
    a = zeros((M,r.size),int8)
    pts = zeros(M,int)
    pts[0] = n if n else r[int(random.uniform(r.size))]
    for i in range(1,M):
        a[i-1] = amin(hamm(pts[newaxis,i-1],rr[:,r]),axis=0)
        pts[i] = r[argmax(amin(a[:i],axis=0), axis=0)]
    i=M
    a[i-1] = amin(hamm(pts[newaxis,i-1],rr[:,r]),axis=0)
    m = matrix([makeSquare(pt).reshape(16) for pt in pts])
    if visual:
        figure(1);clf();spy(m)
        #figure(2);clf();imshow(a,extent=(0,1,0,1))
    on = sum(m,axis=1)
    print pts[0],amin(a[:,where(r==pts[:,newaxis])[1]]+100*eye(M,M,0,int8)),mean(on),std(on)
    return pts,a

def gen_numbers():
    pts,a=attempt()
    open('numbers', 'w').write('\n'.join([str(pt) for pt in pts]))
    return pts

def gen_table_sourcefiles(pts):
    base = sort(rr[:,pts].flatten())

    team = zeros(base.size)
    orientation = zeros(base.size)
    for i,x in enumerate(base):
        source = where(rr[:,pts]==x)
        team[i] = source[1][0]
        orientation[i] = source[0][0]

    b = zeros((65536,2),dtype=int)
    b[:,1] = 32
    b[base,0] = arange(base.size)
    b[base,1] = 0
    def mutate(i):
        return c_[arange(i.size,dtype=int)[:,newaxis], i^(1<<arange(0,16)[newaxis,:])]

    m = mutate(arange(65536)[:,newaxis])
    while sum(b[:,1]==32):
        best = m[(arange(65536),argmin(b[m,1],axis=1))]
        b[:,0] = b[best,0]
        cost = b[best,1]
        b[:,1] = where(cost==32,32,where(best!=arange(65536),cost+1,cost))

    c = zeros((65536,3),dtype=int)
    c[:,0] = team[b[:,0]]
    c[:,1] = orientation[b[:,0]]
    c[:,2] = b[:,1]

    f = open('../vision/table.c','w')
    codes = ['%3d' % x for x in c[:,0]+(c[:,1]<<5)+(c[:,2]<<7)]
    def pretty_print(codes):
        stride = 16
        return '    ' + ',\n    '.join(', '.join(codes[i:i+stride]) for i in xrange(0,len(codes),stride))

    f.write('#include "table.h"\n\n// Generated automatically\n\n')
    f.write('uint16_t hamming_codes[65536] = {\n' + pretty_print(codes) + '\n};\n')
    f.close()

    f = open('../vision/table.h','w')
    f.write('#ifndef _TABLE_H_INC_\n')
    f.write('#define _TABLE_H_INC_\n\n')
    f.write('#include <stdint.h>\n\n')
    f.write('// Generated automatically\n\n')
    f.write('extern uint16_t hamming_codes[65536];\n\n')
    f.write('#define HAMMING_DECODE(_num_, _pid_, _porient_, _pdist_) { \\\n')
    f.write('    int entry = hamming_codes[_num_];  \\\n')
    f.write('    *(_pid_) = entry & 0x1F;           \\\n')
    f.write('    *(_porient_) = (entry & 0x60) >> 5; \\\n')
    f.write('    *(_pdist_) = (entry & 0x380) >> 7; \\\n')
    f.write('}\n\n')
    f.write('#endif\n')
    f.close()

pts = gen_numbers()
gen_table_sourcefiles(pts)
