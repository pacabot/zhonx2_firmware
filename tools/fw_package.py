#!/usr/bin/env python3
"""Build an OTA image and a sparse Intel HEX for initial SWD provisioning.
No device access. Sparse HEX deliberately omits persistence sectors 1 and 2.
"""
import hashlib
import json
from pathlib import Path
import struct
import sys
import zlib

APP_BASE = 0x08010000
IMAGE_MAX = 0x60000


def validate(data):
    if len(data) < 8 or len(data) > IMAGE_MAX or len(data) % 4:
        raise ValueError("Image length must be word-aligned and <=384 KiB")
    sp, pc = struct.unpack_from("<II", data)
    if not (0x20000000 < sp <= 0x20020000 and sp % 8 == 0):
        raise ValueError("Invalid/alignment of initial SRAM stack")
    if not (pc & 1 and APP_BASE <= pc < APP_BASE + len(data)):
        raise ValueError("Image must be linked at 0x08010000 with a Thumb entry")


def manifest(data):
    first = struct.pack("<4I", 0x32474D49, 1, len(data), zlib.crc32(data))
    return first + struct.pack("<4I", zlib.crc32(first), 0xFFFFFFFC, 0x434F4D54, 0xFFFFFFFF)


def ihex(segments):
    lines = []
    def record(address, kind, data):
        body = struct.pack(">BHB", len(data), address, kind) + data
        lines.append(":" + (body + bytes([-sum(body) & 255])).hex().upper())
    for base, data in sorted(segments):
        upper = None
        for off in range(0, len(data), 16):
            addr = base + off
            if addr >> 16 != upper:
                upper = addr >> 16
                record(0, 4, struct.pack(">H", upper))
            record(addr & 65535, 0, data[off:off + 16])
    record(0, 1, b"")
    return "\n".join(lines) + "\n"


def package_app(directory):
    directory = Path(directory)
    data = (directory / "ZHONX_II_M4.bin").read_bytes()
    data += b"\xff" * (-len(data) % 4)
    validate(data)
    (directory / "application.ota.bin").write_bytes(data)
    (directory / "image-manifest.bin").write_bytes(manifest(data))
    return data


def package(directory):
    directory = Path(directory)
    data = package_app(directory)
    boot = (directory / "bootloader.bin").read_bytes()
    if len(boot) > 16384:
        raise ValueError("Bootloader exceeds sector zero")
    # Only initial installation: boot + metadata + app. No stage/saved data.
    (directory / "initial-install.hex").write_text(ihex([
        (0x08000000, boot), (0x0800C000, manifest(data)), (APP_BASE, data)]))
    files = ["application.ota.bin", "bootloader.bin", "initial-install.hex"]
    info = {name: {"bytes": (directory / name).stat().st_size,
                   "sha256": hashlib.sha256((directory / name).read_bytes()).hexdigest()}
            for name in files}
    info["application_address"] = hex(APP_BASE)
    info["application_crc32"] = f"{zlib.crc32(data):08x}"
    (directory / "firmware-manifest.json").write_text(json.dumps(info, indent=2) + "\n")
    print(f"Application {len(data)} bytes; bootloader {len(boot)} bytes; sparse initial-install.hex")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--app":
        data = package_app(sys.argv[2] if len(sys.argv) > 2 else "build")
        print(f"Application {len(data)} bytes")
    else:
        package(sys.argv[1] if len(sys.argv) > 1 else "build")
