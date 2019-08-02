import os
import sys
import socket
import time
import threading

try:
    addr = sys.argv[1]
    rate = int(sys.argv[2])
    dt = float(sys.argv[3])
except:
    print 'usage: mesytest.py ipaddr rate meastime'
    sys.exit(1)

os.system('python mesyparams.py %s %s' % (addr, rate))

n = [0, 0, 0, 0]

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('', 54321))
s.sendto('\x00\x00\x00\x01\x00\x00\x00\x00' + '\x01\x00' + '\x00' * 10, (addr, 54321))
s.recvfrom(2000)

def get():
    while True:
        d, _ = s.recvfrom(2000)
        if d[3] == '\x01':
            d = map(ord, d)
            n[2] = d[12] | (d[13] << 8) | (d[14] << 16) | (d[15] << 24) | (d[16] << 32) | (d[17] << 40)
            return
        n[0] += 1
        blen = ord(d[0]) | (ord(d[1]) << 8)
        n[1] += (blen - 21) / 3
        n[3] += len(d) + 66  # Eth/IP/UDP headers

t = threading.Thread(target=get)
t.setDaemon(True)
t.start()
time.sleep(dt)
s.sendto('\x00\x00\x00\x01' + '\x00' * 16, (addr, 54321))
t.join()

print 'got: %s packets with %s events => %.0f ev/s [set: %.0f], %.0f ev/pkt, %.2f MB/s' % \
    (n[0], n[1], n[1]/(n[2] / 1e7), 10000000//(10000000//rate), n[1]/float(n[0]), (n[3])/float(n[2] / 10.))
