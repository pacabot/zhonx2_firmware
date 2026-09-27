"""Exercise the real flash/build entry points with hardware tools replaced."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FlashBuildTest(unittest.TestCase):
    def run_script(self, arguments, failure=False):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ('scripts/flash/flash.sh', 'scripts/build/build.sh'):
                target = root / name
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(ROOT / name, target)
                target.chmod(0o755)
            binary = root / 'bin'
            binary.mkdir()
            log = root / 'calls.log'
            for tool in ('make', 'python3', 'openocd', 'arm-none-eabi-nm',
                         'arm-none-eabi-gcc', 'arm-none-eabi-objcopy', 'arm-none-eabi-size'):
                target = binary / tool
                target.write_text('#!/bin/sh\n'
                                  'printf "%s %s\\n" "${0##*/}" "$*" >> "$CALL_LOG"\n'
                                  'if [ "${0##*/}" = make ]; then exit "$BUILD_EXIT"; fi\n'
                                  'exit 0\n')
                target.chmod(0o755)
            env = dict(os.environ, PATH=str(binary)+os.pathsep+os.environ['PATH'],
                       CALL_LOG=str(log), BUILD_EXIT='2' if failure else '0',
                       SWD_KHZ='1000', STLINK_SERIAL='', JOBS='1')
            result = subprocess.run(['bash', str(root/'scripts/flash/flash.sh'), *arguments],
                                    cwd=directory, env=env, text=True, capture_output=True)
            return result, log.read_text().splitlines() if log.exists() else []

    def test_profile_and_target_built_before_hardware(self):
        for args, profile, target, optimization in (
            (['--release', '--fast'], 'release', 'app', '-O3'),
            ([], 'debug', 'app', '-Og'),
            (['--release', '--bootloader', '--fast'], 'release', 'boot', '-O3'),
            (['--debug', '--all', '--verify'], 'debug', 'all', '-Og'),
        ):
            with self.subTest(args=args):
                result, calls = self.run_script(args)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertTrue(calls[0].startswith('make '))
                self.assertIn('BUILD=build/'+profile, calls[0])
                self.assertIn('OPT='+optimization, calls[0])
                self.assertTrue(calls[0].endswith(' '+target))
                self.assertNotIn('--fast', calls[0])
                self.assertTrue(any(c.startswith('openocd ') for c in calls))

    def test_build_failure_never_contacts_target(self):
        result, calls = self.run_script(['--release', '--fast'], failure=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(len(calls), 1)
        self.assertTrue(calls[0].startswith('make '))
        self.assertIn('programmation annulée', result.stderr)

    def test_help_does_not_build_or_flash(self):
        result, calls = self.run_script(['--help'])
        self.assertEqual(result.returncode, 0)
        self.assertEqual(calls, [])
