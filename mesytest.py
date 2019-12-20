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
    rtstart = 0

    @classmethod
    def out(cls):
        rtend = time.time() - cls.rtstart
        print('[%9f] got: %s packets with %s events => %.0f ev/s [set: %.0f], '
              '%.0f ev/pkt, %.1f MB/s' %
              (rtend, cls.npackets, cls.nevents, cls.nevents / rtend,
               10000000 // (10000000 // rate),
               cls.nevents / float(cls.npackets),
               cls.nbytes / float(rtend * 1e6)))


def get():
    lastno = None
    while True:
        data, _ = sock.recvfrom(2000)
        t1, t2 = struct.unpack('IH', data[12:18])
        result.endtime = (t1 | (t2 << 32)) / 1e7
        if data[3:4] == b'\x80':  # command reply, i.e. "stop acknowledged"
            break
        result.npackets += 1
        nwords = ord(data[0:1]) | (ord(data[1:2]) << 8)
        result.nevents += (nwords - 21) / 3
        result.nbytes += len(data) + 66  # Eth/IP/UDP header overhead
        bufno = ord(data[6:7]) | (ord(data[7:8]) << 8)
        if not (lastno is None or bufno == (lastno + 1) % 65536):
            print("!!! missing buffer: %s %s" % (bufno, lastno))
        lastno = bufno


def periodic_out(thread):
    while thread.is_alive():
        time.sleep(0.5)
        result.out()


thread = threading.Thread(target=get)
thread.daemon = True
periodic_thread = threading.Thread(target=periodic_out, args=(thread,))
periodic_thread.daemon = True
send_cmd(sock, addr, 1, 'H', 0)
result.rtstart = time.time()
thread.start()
periodic_thread.start()
try:
    time.sleep(dt)
except KeyboardInterrupt:
    pass
send_cmd(sock, addr, 0, 'H', 0, reply=False)
thread.join()
periodic_thread.join()

result.out()
