import struct


PORT = 54321


def send_cmd(sock, addr, cmdno, fmt, *data, **kwds):
    ndata = struct.calcsize(fmt) // 2
    pkt = struct.pack('<HHHHH10x' + fmt, 10 + ndata, 0x8000, 10, 0, cmdno, *data)
    sock.sendto(pkt, (addr, PORT))
    if kwds.get('reply', True):
        sock.recvfrom(1024)
