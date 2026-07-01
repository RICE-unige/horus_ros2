#!/usr/bin/env python3
# Copyright 2026 RICE Lab, University of Genoa
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
from pathlib import Path
import struct
import subprocess
import sys
import unittest


SOURCE_DIR = Path(sys.argv[1]) if len(sys.argv) > 1 else None


class HorusLinkFrameInspectorTest(unittest.TestCase):
    def setUp(self):
        if SOURCE_DIR is None:
            raise RuntimeError('expected source directory argument')
        self.source_dir = SOURCE_DIR
        self.inspector = self.source_dir / 'tools' / 'horuslink_frame_inspector.py'

    def run_inspector(self, payload):
        result = subprocess.run(
            [sys.executable, str(self.inspector), '-'],
            input=payload,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=True,
        )
        return json.loads(result.stdout.decode('utf-8'))

    def test_decodes_hello_control_frame(self):
        hello = bytes.fromhex(
            (self.source_dir / 'test' / 'golden_vectors' / 'hello.hex').read_text()
        )
        frame = struct.pack('<HBBIII', 0, 2, 0, 1, 0, len(hello)) + hello

        decoded = self.run_inspector(frame)

        self.assertEqual(len(decoded), 1)
        self.assertEqual(decoded[0]['msg_type'], 'CONTROL')
        self.assertEqual(decoded[0]['length'], len(hello))
        kind_records = [
            record for record in decoded[0]['tlv'] if record['name'] == 'kind'
        ]
        self.assertEqual(kind_records[0]['value'], 'HELLO')

    def test_reports_truncated_payload(self):
        frame = struct.pack('<HBBIII', 3, 1, 1, 9, 0, 8) + b'abc'

        result = subprocess.run(
            [sys.executable, str(self.inspector), '-'],
            input=frame,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )

        self.assertEqual(result.returncode, 1)
        decoded = json.loads(result.stdout.decode('utf-8'))
        self.assertIn('truncated payload', decoded[0]['error'])


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]])
