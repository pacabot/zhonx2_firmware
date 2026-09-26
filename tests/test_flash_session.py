"""Check whole-flash comparison, including variable-size erase boundaries."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location(
    "flash_session", Path(__file__).resolve().parents[1] / ".scripts/build/flash_session.py")
flash = importlib.util.module_from_spec(spec)
spec.loader.exec_module(flash)


class FlashSessionTest(unittest.TestCase):
    def test_readback_and_preserved_sectors(self):
        for length, erase_end in [(0xD6D4, 0x20000), (0x10004, 0x40000),
                                  (0x60000, 0x80000)]:
            with self.subTest(length=length), tempfile.TemporaryDirectory() as directory:
                session = Path(directory)
                before = b'\xa5' * flash.FLASH_SIZE
                (session / 'pre-flash.bin').write_bytes(before)
                (session / 'bootloader.bin').write_bytes(b'\x12' * 3632)
                (session / 'image-manifest.bin').write_bytes(b'\x34' * 32)
                (session / 'application.ota.bin').write_bytes(b'\x56' * length)
                expected = bytearray(before)
                expected[:0x4000] = b'\xff' * 0x4000
                expected[0xC000:erase_end] = b'\xff' * (erase_end - 0xC000)
                for name, offset in flash.IMAGES:
                    data = (session / name).read_bytes()
                    expected[offset:offset + len(data)] = data
                (session / 'post-flash.bin').write_bytes(expected)
                self.assertTrue(flash.verify(session)['untouched_flash_verified'])
                # A changed byte in an image, persistence, stage, reserved or
                # erased padding must all reject the readback.
                for offset in (0, 0x4000, 0x8000, 0x10000, 0x80000, 0xE0000, 0x3FFF):
                    corrupted = expected.copy()
                    corrupted[offset] ^= 1
                    (session / 'post-flash.bin').write_bytes(corrupted)
                    with self.assertRaises(ValueError):
                        flash.verify(session)
                (session / 'post-flash.bin').write_bytes(expected[:-1])
                with self.assertRaises(ValueError):
                    flash.verify(session)
