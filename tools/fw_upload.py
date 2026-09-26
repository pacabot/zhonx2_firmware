#!/usr/bin/env python3
"""Upload through a future TCP-to-half-duplex-SPI bridge (no module assumed).
The bridge maps each 1056-byte TCP request to one SPI request and returns 32 bytes.
"""
import argparse
from pathlib import Path
import socket
import struct
import time
import zlib
from fw_package import validate

MAGIC = 0x3257465A


def packet(command, sequence, arg0=0, arg1=0, data=b""):
    if len(data) > 1024:
        raise ValueError("Payload exceeds 1024 bytes")
    head = struct.pack("<7I", MAGIC, command, sequence, arg0, arg1, len(data), 1)
    return head + struct.pack("<I", zlib.crc32(data, zlib.crc32(head))) + data.ljust(1024, b"\xff")


def receive(sock, count):
    data = bytearray()
    while len(data) < count:
        part = sock.recv(count - len(data))
        if not part:
            raise ConnectionError("Bridge disconnected; reset/restart upload")
        data += part
    return bytes(data)


def exchange(sock, command, sequence, arg0=0, arg1=0, data=b""):
    sock.sendall(packet(command, sequence, arg0, arg1, data))
    reply = receive(sock, 32)
    words = struct.unpack("<8I", reply)
    if words[0] != MAGIC or words[1] != sequence or words[6] != 1 or words[7] != zlib.crc32(reply[:28]):
        raise ValueError("Invalid reply, sequence, or CRC; restart upload")
    if words[2]:
        raise RuntimeError(f"Loader rejected command {command} ({words[2]:08x})")
    return words


def upload(sock, data, boot=True):
    validate(data)
    info = exchange(sock, 1, 0)
    if info[5] != 0x08010000 or len(data) > info[3]:
        raise ValueError("Incompatible bootloader layout")
    exchange(sock, 2, 1, len(data), zlib.crc32(data))
    seq = 2
    for off in range(0, len(data), 1024):
        chunk = data[off:off + 1024]
        reply = exchange(sock, 3, seq, off, data=chunk)
        if reply[4] != off + len(chunk):
            raise ValueError("Loader offset mismatch")
        seq += 1
    exchange(sock, 4, seq)  # Full CRC + installation + verification before ACK.
    if boot:
        exchange(sock, 5, seq + 1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("host", help="Address of a bridge implementing docs/FIRMWARE.md")
    parser.add_argument("--port", type=int, default=9020)
    parser.add_argument("--stay", action="store_true", help="Stay in bootloader after verification")
    args = parser.parse_args()
    data = args.image.read_bytes()
    validate(data)
    start = time.monotonic()
    with socket.create_connection((args.host, args.port), timeout=30) as sock:
        sock.settimeout(60)  # STM32 sector erase/program dominates transfer time.
        upload(sock, data, not args.stay)
    print(f"Verified and installed {len(data)} bytes in {time.monotonic()-start:.2f}s")


if __name__ == "__main__":
    main()
