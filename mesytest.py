import sys
import socket
import struct
import threading
import time

from mesylib import PORT, send_cmd

try:
    addr = sys.argv[1]
    rate = int(sys.argv[2])
    dt = float(sys.argv[3])
except (ValueError, IndexError):
    print('usage: mesytest.py ipaddr rate meastime')
    sys.exit(1)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', PORT))
send_cmd(sock, addr, 0xF1F0, 'IH', rate, 0)
print('configure ok')


class result:
    npackets = 0
    nevents = 0
    nbytes = 0
    endtime = 0


def get():
    while True:
        data, _ = sock.recvfrom(2000)
        if data[3:4] == b'\x80':  # command reply, i.e. "stop acknowledged"
            t1, t2 = struct.unpack('IH', data[12:18])
            result.endtime = (t1 | (t2 << 32)) / 1e7
            break
        result.npackets += 1
        nwords = ord(data[0:1]) | (ord(data[1:2]) << 8)
        result.nevents += (nwords - 21) / 3
        result.nbytes += len(data) + 66  # Eth/IP/UDP headers


thread = threading.Thread(target=get)
send_cmd(sock, addr, 1, 'H', 0)
thread.start()
time.sleep(dt)
send_cmd(sock, addr, 0, 'H', 0, reply=False)
thread.join()

print('got: %s packets with %s events => %.0f ev/s [set: %.0f], %.0f ev/pkt, %.1f MB/s' %
      (result.npackets,
       result.nevents,
       result.nevents / result.endtime,
       10000000 // (10000000 // rate),
       result.nevents / float(result.npackets),
       result.nbytes / float(result.endtime * 1e6)))
