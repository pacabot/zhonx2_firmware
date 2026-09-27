import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

spec=importlib.util.spec_from_file_location('robot_guard',Path(__file__).resolve().parents[1]/'scripts/flash/robot_guard.py')
guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)
SERIAL='51FF66064982565324552187'
OTHER='55FF68064967515341421587'
UID='001100223344556677889900'
ENTRY=dict(serial=SERIAL,uid=UID)


class RobotGuardTest(unittest.TestCase):
    def test_unbound_and_wrong_override_refused(self):
        with tempfile.TemporaryDirectory() as folder, patch.dict(os.environ, XDG_CONFIG_HOME=folder, STLINK_SERIAL=''):
            with self.assertRaisesRegex(ValueError,'non associé'):guard.identity('zhonx2')
            with patch.object(guard,'inspect_probe',return_value=UID):guard.associate('zhonx2',SERIAL)
            self.assertEqual(guard.identity('zhonx2'),ENTRY)
            with patch.dict(os.environ,STLINK_SERIAL=OTHER):
                with self.assertRaisesRegex(ValueError,'ne correspond pas'):guard.identity('zhonx2')
            before=guard.registry_path().read_bytes()
            with patch.object(guard,'inspect_probe',return_value='0123456789ABCDEF01234567'):
                with self.assertRaisesRegex(ValueError,'STM32 a changé'):guard.associate('zhonx2',SERIAL)
            self.assertEqual(guard.registry_path().read_bytes(),before)

    def test_duplicate_probe_or_robot_refused(self):
        for entry in (dict(serial=SERIAL,uid='0123456789ABCDEF01234567'),dict(serial=OTHER,uid=UID)):
            with self.assertRaisesRegex(ValueError,'ambiguë'):
                guard.validate_registry(dict(schema=1,robots={'zhonx2':dict(ENTRY),'zhonx3':entry}))
        for serial in ('', '123', SERIAL+';shutdown'):
            with self.assertRaises(ValueError):guard.base_config(serial)

    def test_uid_check_stops_tcl_before_programming(self):
        # Execute the emitted Tcl with fake memory reads; never invoke OpenOCD.
        for uid,accepted in ((UID,True),('0025003B3332471930303430',False)):
            words=' '.join('0x'+uid[i:i+8] for i in range(0,24,8))
            prelude='''proc find {path} {return $path}
proc source args {}
proc adapter args {}
proc stm32f4x.cpu args {}
proc transport args {}
proc gdb_port args {}
proc tcl_port args {}
proc telnet_port args {}
proc init args {}
proc echo args {puts [join $args]}
proc read_memory {address width count} {
 if {$address == 0xe0042000} {return 0x413}
 if {$address == 0x1fff7a22} {return 1024}
 return {WORDS}
}
'''.replace('WORDS',words)
            with tempfile.TemporaryDirectory() as directory:
                path=Path(directory)/'test.tcl'
                path.write_text(prelude+guard.guarded_config('zhonx2',ENTRY)+'puts PROGRAM_ALLOWED\n')
                result=subprocess.run(['tclsh',str(path)],capture_output=True,text=True)
            self.assertEqual(result.returncode==0,accepted,result.stderr)
            self.assertEqual('PROGRAM_ALLOWED' in result.stdout,accepted)
            if not accepted:self.assertIn('Mauvais robot',result.stderr)

    def test_inspection_does_not_reset_halt_or_write(self):
        def fake_run(command,**kwargs):
            config=Path(command[-1]).read_text()
            self.assertIn('adapter serial '+SERIAL,config)
            for forbidden in ('reset','halt','flash write','mww','program'):
                self.assertNotRegex(config,r'(?m)^\s*'+forbidden+r'\b')
            return subprocess.CompletedProcess(command,0,'ROBOT_UID:'+UID+'\n','')
        with patch.object(guard.subprocess,'run',side_effect=fake_run):
            self.assertEqual(guard.inspect_probe(SERIAL),UID)
