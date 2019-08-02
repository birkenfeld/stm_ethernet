import socket
import sys
import time

addr = sys.argv[1]

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('', 12345))
s.sendto('start', (addr, 50000))
t0 = None
ll = []
pktl = 0

for i in range(10*1000):
    b, _ = s.recvfrom(2000)
    if t0 is None:
        print 'start'
        pktl = len(b) + 66
        t0 = time.time()
    n = ord(b[0])<<24 | ord(b[1])<<16 | ord(b[2])<<8 | ord(b[3])
    ll.append(n)
s.sendto('stop', (addr, 50000))
t = time.time() - t0
print 'Took %.3f s to receive 10M pkts (%.1f MBit/s)' % (t, pktl/12.5/t)
assert ll == range(10*1000)
