import sys
import socket
import struct
import time

addr = sys.argv[1]

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 12345))

t0 = None
seq_nos = []
packet_len = 0

sock.sendto(b'start', (addr, 50000))

for _ in range(10*1000):
    data, _ = sock.recvfrom(2000)
    if t0 is None:
        t0 = time.time()
        print('started')
        # 66 bytes are Ethernet/IP/UDP overhead
        packet_len = len(data) + 66
    seq_nos.append(struct.unpack('>I', data[:4])[0])

total_time = time.time() - t0
sock.sendto(b'stop', (addr, 50000))

print('Took %.3f s to receive 10M pkts (%.1f MBit/s)' %
      (total_time, packet_len/12.5/total_time))
assert seq_nos == list(range(10*1000))
