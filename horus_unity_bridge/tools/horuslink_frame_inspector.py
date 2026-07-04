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

"""Inspect HorusLink/1 binary frames.

The tool accepts either raw binary captures or whitespace-separated hex. It is
intentionally dependency-free so it can run on robots, WSL, or CI while
debugging the Unity <-> bridge link.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import struct
import sys
from typing import Iterable


HEADER_SIZE = 16

MESSAGE_TYPES = {
    1: 'DATA',
    2: 'CONTROL',
    3: 'SVC_REQ',
    4: 'SVC_RESP',
    5: 'KEEPALIVE',
    6: 'FLOW',
}

FLAGS = {
    1 << 0: 'RAW_OPAQUE',
    1 << 1: 'BEGIN',
    1 << 2: 'END',
    1 << 3: 'REPLACE_LATEST',
    1 << 4: 'COMPRESSED',
}

CONTROL_KINDS = {
    1: 'HELLO',
    2: 'SUBSCRIBE_REQUEST',
    3: 'SUBSCRIBE_ACK',
    4: 'TOPIC_TABLE',
    5: 'PUBLISHER_REQUEST',
    6: 'SERVICE_CLIENT_REQUEST',
    7: 'CHANNEL_CLOSE_REQUEST',
    8: 'TOPIC_TABLE_REQUEST',
}

TLV_NAMES = {
    1: 'kind',
    2: 'protocol_version',
    3: 'role',
    4: 'channel_id',
    5: 'topic',
    6: 'type_name',
    7: 'lane',
    8: 'delivery',
    9: 'max_payload',
    10: 'keepalive_ms',
    11: 'subscribe_status',
    12: 'error',
    13: 'session_id',
    20: 'topic_entry',
}


def decode_flags(value: int) -> list[str]:
    names = [name for bit, name in FLAGS.items() if value & bit]
    unknown = value & ~sum(FLAGS.keys())
    if unknown:
        names.append(f'UNKNOWN_0x{unknown:02x}')
    return names


def load_input(path: str | None, hex_mode: bool) -> bytes:
    data = sys.stdin.buffer.read() if path in (None, '-') else Path(path).read_bytes()
    if not hex_mode:
        return data
    text = data.decode('utf-8')
    compact = re.sub(r'[^0-9a-fA-F]', '', text)
    if len(compact) % 2:
        raise ValueError('hex input contains an odd number of digits')
    return bytes.fromhex(compact)


def decode_scalar(value: bytes) -> int | str:
    if len(value) == 1:
        return value[0]
    if len(value) == 2:
        return struct.unpack_from('<H', value)[0]
    if len(value) == 4:
        return struct.unpack_from('<I', value)[0]
    if len(value) == 8:
        return struct.unpack_from('<Q', value)[0]
    try:
        decoded = value.decode('utf-8')
    except UnicodeDecodeError:
        return value.hex()
    return decoded if decoded.isprintable() else value.hex()


def decode_tlvs(payload: bytes) -> tuple[list[dict[str, object]], str | None]:
    records: list[dict[str, object]] = []
    offset = 0
    while offset < len(payload):
        if len(payload) - offset < 4:
            return records, f'truncated TLV header at offset {offset}'
        tlv_type, length = struct.unpack_from('<HH', payload, offset)
        offset += 4
        if len(payload) - offset < length:
            return records, f'truncated TLV value at offset {offset}'
        value = payload[offset:offset + length]
        offset += length
        decoded_value: object = decode_scalar(value)
        if tlv_type == 1 and isinstance(decoded_value, int):
            decoded_value = CONTROL_KINDS.get(decoded_value, f'UNKNOWN_{decoded_value}')
        records.append(
            {
                'type': tlv_type,
                'name': TLV_NAMES.get(tlv_type, f'unknown_{tlv_type}'),
                'length': length,
                'value': decoded_value,
                'hex': value.hex(),
            }
        )
    return records, None


def iter_frames(data: bytes) -> Iterable[dict[str, object]]:
    offset = 0
    frame_index = 0
    while offset < len(data):
        if len(data) - offset < HEADER_SIZE:
            yield {
                'index': frame_index,
                'offset': offset,
                'error': f'truncated frame header: {len(data) - offset} bytes remain',
            }
            return

        header = data[offset:offset + HEADER_SIZE]
        channel_id, msg_type, flags, seq, corr_id, length = struct.unpack('<HBBIII', header)
        offset += HEADER_SIZE
        if len(data) - offset < length:
            yield {
                'index': frame_index,
                'offset': offset - HEADER_SIZE,
                'channel_id': channel_id,
                'msg_type': MESSAGE_TYPES.get(msg_type, f'UNKNOWN_{msg_type}'),
                'flags': decode_flags(flags),
                'seq': seq,
                'corr_id': corr_id,
                'length': length,
                'error': f'truncated payload: expected {length}, have {len(data) - offset}',
            }
            return

        payload = data[offset:offset + length]
        offset += length

        frame: dict[str, object] = {
            'index': frame_index,
            'offset': offset - length - HEADER_SIZE,
            'channel_id': channel_id,
            'msg_type': MESSAGE_TYPES.get(msg_type, f'UNKNOWN_{msg_type}'),
            'flags': decode_flags(flags),
            'seq': seq,
            'corr_id': corr_id,
            'length': length,
            'payload_preview': payload[:32].hex(),
        }
        if msg_type == 2:
            tlvs, error = decode_tlvs(payload)
            frame['tlv'] = tlvs
            if error:
                frame['error'] = error
        yield frame
        frame_index += 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path', nargs='?', help='Frame capture path, or - / omitted for stdin.')
    parser.add_argument(
        '--hex',
        action='store_true',
        help='Treat input as whitespace-separated hex.',
    )
    parser.add_argument('--pretty', action='store_true', help='Pretty-print JSON output.')
    args = parser.parse_args()

    try:
        data = load_input(args.path, args.hex)
        frames = list(iter_frames(data))
    except Exception as exc:  # noqa: BLE001 - command-line diagnostic path
        print(json.dumps({'error': str(exc)}), file=sys.stderr)
        return 2

    print(json.dumps(frames, indent=2 if args.pretty else None))
    return 0 if not any('error' in frame for frame in frames) else 1


if __name__ == '__main__':
    raise SystemExit(main())
