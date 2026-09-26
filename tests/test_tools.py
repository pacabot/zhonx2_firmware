import importlib.util
from pathlib import Path
import struct
import sys
import unittest
import zlib
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import fw_package
import fw_upload

class ToolsTest(unittest.TestCase):
    def image(self):
        return struct.pack('<II', 0x20020000, 0x08010009) + bytes(56)

    def test_image_vectors(self):
        fw_package.validate(self.image())
        for data in [b'', self.image()[:-1], struct.pack('<II',0x20000004,0x08010009)+bytes(56),
                     struct.pack('<II',0x20020000,0x08000009)+bytes(56)]:
            with self.assertRaises(ValueError): fw_package.validate(data)

    def test_packet_crc(self):
        p = fw_upload.packet(3, 99, 1024, data=b'abcd')
        self.assertEqual(len(p),1056)
        words = struct.unpack('<8I',p[:32])
        self.assertEqual(words[7],zlib.crc32(p[32:36],zlib.crc32(p[:28])))
        self.assertEqual(words[2],99)

    def test_manifest(self):
        m=fw_package.manifest(self.image())
        words=struct.unpack('<8I',m)
        self.assertEqual(words[3],zlib.crc32(self.image()))
        self.assertEqual(words[4],zlib.crc32(m[:16]))
        self.assertEqual(words[5],0xfffffffc)

    def test_sparse_hex(self):
        text=fw_package.ihex([(0x08000000,b'boot'),(0x0800c000,bytes(32)),(0x08010000,self.image())])
        addresses=[]; upper=0
        for line in text.splitlines():
            b=bytes.fromhex(line[1:]); self.assertEqual(sum(b)&255,0)
            if b[3]==4: upper=int.from_bytes(b[4:6],'big')<<16
            if b[3]==0:
                address=upper+int.from_bytes(b[1:3],'big')
                addresses.extend(range(address,address+b[0]))
        self.assertTrue(all(not 0x08004000<=a<0x0800c000 for a in addresses))
        self.assertIn(0x08010000,addresses)

    def test_upload_fragmented_reply(self):
        class Bridge:
            reply=b''
            commands=[]
            def sendall(self,p):
                w=struct.unpack('<8I',p[:32]); self.commands.append(w[1])
                h=struct.pack('<7I',fw_upload.MAGIC,w[2],0,0x60000,
                              w[3]+w[5] if w[1]==3 else 0,0x08010000,1)
                self.reply=h+struct.pack('<I',zlib.crc32(h))
            def recv(self,n):
                chunk=self.reply[:min(n,3)]; self.reply=self.reply[len(chunk):]; return chunk
        b=Bridge(); fw_upload.upload(b,self.image())
        self.assertEqual(b.commands,[1,2,3,4,5])

if __name__ == '__main__': unittest.main()
