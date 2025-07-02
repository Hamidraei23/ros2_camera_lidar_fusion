#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Ultra-light UDP JPEG receiver / viewer.
Usage:  python3 udp_image_receiver.py --port 5005
Press 'q' to quit.
"""

import argparse, socket, cv2, numpy as np

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ip',   default='0.0.0.0',  help='interface to bind')
    ap.add_argument('--port', type=int, default=5005)
    ap.add_argument('--show', action='store_true', help='display window (default on)')
    args = ap.parse_args()
    args.show = True if args.show or not ap.get_default('show') else False

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.ip, args.port))
    print(f'🟢  Listening on {args.ip}:{args.port}')

    while True:
        data, _ = sock.recvfrom(65536)               # single datagram
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            continue                                 # corrupt packet
        if args.show:
            cv2.imshow('UDP Stream', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    sock.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
