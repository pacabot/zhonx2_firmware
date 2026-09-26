import importlib.util
from pathlib import Path
import struct
import unittest
import zlib

spec = importlib.util.spec_from_file_location('caldump', Path(__file__).parents[1]/'tools/calibration_dump.py')
dump = importlib.util.module_from_spec(spec);spec.loader.exec_module(dump)


def bank(schema, seq, payload):
    first = struct.pack('<5I',0x3254535a,schema,seq,len(payload),zlib.crc32(payload))
    return first+struct.pack('<3I',zlib.crc32(first),0x434f4d54,0xffffffff)+payload


class CalibrationDumpTest(unittest.TestCase):
    def test_crc_generation_and_signed_offsets(self):
        payload = bytearray(1244)
        struct.pack_into('<18I',payload,412,1,3,47000,94000,167000,179000,
                         96000,100000,100,145000,148000,200,78500,79500,0,88500,89500,0)
        corner = 412+72+128
        struct.pack_into('<7I',payload,corner,1,0,173000,47000,94000,167000,179000)
        struct.pack_into('<2I8i2I',payload,corner+28,40,3,-15000,-25000,-12000,-20000,
                         -14500,-24000,-13000,-21000,1000,2000)
        memory=bytearray(b'\xff'*0x100000)
        a=bank(4,0xffffffff,payload);b=bank(5,0,payload+bytes(3456))
        memory[0x4000:0x4000+len(a)]=a;memory[0x8000:0x8000+len(b)]=b
        report=dump.decode(memory)
        self.assertEqual(report['selected_bank'],'B')
        self.assertEqual(report['wall']['geometry']['width_mm'],94)
        self.assertEqual(report['corners'][0]['profiles'][0]['raw_open_mm'],[-15,-25])
        memory[0x8020+100]^=1
        self.assertEqual(dump.decode(memory)['selected_bank'],'A')
        self.assertEqual(dump.decode(memory[0x4000:0xC000])['selected_bank'],'A')
        memory[0x4020+100]^=1
        self.assertIn('error',dump.decode(memory))
