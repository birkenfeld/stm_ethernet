import socket, sys, struct

try:
    addr = sys.argv[1]
    rate = int(sys.argv[2])
except (ValueError, IndexError):
    print 'usage: mesyparams.py ipaddr rate'
else:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ndata = 3  # number of words
    pkt = struct.pack(
        '<HHHHHBBHHHH' + 'IH',
        10 + ndata,
        0x8000,
        10,
        0,
        0xF1F0,
        0,
        0,
        0,
        0,
        0,
        0,
        rate,
        0,
    )
    s.sendto(pkt, (addr, 54321))
    s.recvfrom(1024)
    print 'configure ok'
