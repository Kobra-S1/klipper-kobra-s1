#!/usr/bin/env python3
"""
(c) 2026 Antiriad
This file may be distributed under the terms of the GNU GPLv3 license.

Anycubic Kobra Klipper MCU OTA compatible Firmware Update Tool
Based on Anycubic's open-source OTA MCU implementation and Go-Klipper's OTA protocol.

!!!!! WARNING!!!!! !!!!! WARNING!!!!! !!!!! WARNING!!!!!
This tool can brick your MCU if used incorrectly or things go wrong.
If you flash incompatible or buggy firmware, interrupt the process, or if the MCU is not responsive to the OTA protocol,
you may need to use a hardware programmer to recover.
!!!!! WARNING!!!!! !!!!! WARNING!!!!! !!!!! WARNING!!!!!

Flashing Implementation Notes:
1. CRC is last 8 ASCII hex bytes (not 9)
2. Strip trailing newline during transfer (not during CRC calc)
3. Empty data response triggers CRC check on MCU
4. No reset command - MCU NEEDS TO RESTARTED MANUALLY after OTA completes (no auto-reconnect logic)
"""

import argparse
import json
from pathlib import Path
import re
import serial
import subprocess
import sys
import time
import zlib

CHUNK_SIZE = 0x28
TIMEOUT = 1.0
MIN_BLOCK_LEN = 5
MAX_BLOCK_LEN = 96
OTA_OID = 0

cmd_id_map = {}
resp_id_to_name = {}


def crc16_ccitt(buf: bytes) -> int:
    """Calculate CRC16-CCITT checksum"""
    crc = 0xFFFF
    for b in buf:
        data = b ^ (crc & 0xFF)
        data ^= (data << 4) & 0xFF
        crc = (((data << 8) & 0xFFFF) | (crc >> 8)) ^ (data >> 4) ^ ((data << 3) & 0xFFFF)
        crc &= 0xFFFF
    return crc


def vlq_encode(v: int) -> bytes:
    """Encode integer as Variable Length Quantity"""
    sv = v
    out = bytearray()
    if -(1 << 5) <= sv < (3 << 5):
        out.append(v & 0x7F)
    elif -(1 << 12) <= sv < (3 << 12):
        out.extend([((v >> 7) & 0x7F) | 0x80, v & 0x7F])
    elif -(1 << 19) <= sv < (3 << 19):
        out.extend([((v >> 14) & 0x7F) | 0x80, ((v >> 7) & 0x7F) | 0x80, v & 0x7F])
    elif -(1 << 26) <= sv < (3 << 26):
        out.extend([
            ((v >> 21) & 0x7F) | 0x80,
            ((v >> 14) & 0x7F) | 0x80,
            ((v >> 7) & 0x7F) | 0x80,
            v & 0x7F,
        ])
    else:
        out.extend([
            ((v >> 28) & 0x7F) | 0x80,
            ((v >> 21) & 0x7F) | 0x80,
            ((v >> 14) & 0x7F) | 0x80,
            ((v >> 7) & 0x7F) | 0x80,
            v & 0x7F,
        ])
    return bytes(out)


def vlq_decode(buf: bytes, pos: int = 0):
    """Decode Variable Length Quantity at position"""
    if pos >= len(buf):
        raise ValueError("Buffer too short for VLQ decode")
    c = buf[pos]
    pos += 1
    v = c & 0x7F
    if (c & 0x60) == 0x60:
        v |= -0x20
    while c & 0x80:
        if pos >= len(buf):
            raise ValueError("Truncated VLQ data")
        c = buf[pos]
        pos += 1
        v = (v << 7) | (c & 0x7F)
    return v, pos


def build_block(cmd_id: int, params, seq: int) -> bytes:
    """Build a Klipper protocol message block"""
    content = bytearray(vlq_encode(cmd_id))
    for p in params:
        if isinstance(p, int):
            content.extend(vlq_encode(p))
        elif isinstance(p, bytes):
            content.extend(vlq_encode(len(p)))
            content.extend(p)
        else:
            raise TypeError("Unsupported param type")
    length = 2 + len(content) + 3
    header = bytes([length, 0x10 | (seq & 0x0F)])
    body = header + content
    crc = crc16_ccitt(body).to_bytes(2, "big")
    return body + crc + b"\x7E"


def _read_with_deadline(ser: serial.Serial, count: int, deadline: float) -> bytes:
    out = bytearray()
    while len(out) < count:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        ser.timeout = max(0.05, remaining)
        chunk = ser.read(count - len(out))
        if not chunk:
            continue
        out.extend(chunk)
    return bytes(out)


def read_block(ser: serial.Serial, timeout=TIMEOUT) -> bytes:
    """Read one complete message block from serial, resyncing if needed."""
    deadline = time.monotonic() + timeout
    discarded = 0

    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            if discarded:
                raise TimeoutError(f"No valid frame after discarding {discarded} bytes")
            raise TimeoutError("No length byte")

        ser.timeout = max(0.05, remaining)
        first = ser.read(1)
        if not first:
            continue

        length = first[0]
        if length < MIN_BLOCK_LEN or length > MAX_BLOCK_LEN:
            discarded += 1
            continue

        rest = _read_with_deadline(ser, length - 1, deadline)
        if len(rest) != length - 1:
            discarded += 1
            continue

        block = first + rest
        if block[-1] != 0x7E or block[0] != len(block):
            discarded += len(block)
            continue

        if discarded:
            print(f"Resynchronized serial stream after discarding {discarded} byte(s)")
        return block


def parse_identify_response(block: bytes) -> bytes:
    """Parse identify_response and extract compressed dictionary data"""
    # Validate basic framing
    if block[-1] != 0x7E:
        raise ValueError(f"Missing frame terminator: 0x{block[-1]:02X}")
    
    if block[0] != len(block):
        raise ValueError(f"Length mismatch: header says {block[0]}, got {len(block)}")
    
    # Verify CRC
    recv = int.from_bytes(block[-3:-1], "big")
    calc = crc16_ccitt(block[:-3])
    
    if recv != calc:
        print(f"DEBUG: Block hex: {block.hex()}")
        print(f"DEBUG: Received CRC: 0x{recv:04X}, Calculated: 0x{calc:04X}")
        raise ValueError("CRC mismatch")
    
    # Parse content
    content = block[2:-3]
    resp_id, pos = vlq_decode(content, 0)
    if resp_id not in (0, 1):
        raise ValueError(f"Not identify_response (got resp_id={resp_id})")
    _, pos = vlq_decode(content, pos)
    data_len, pos = vlq_decode(content, pos)
    data = content[pos:pos + data_len]
    if len(data) != data_len:
        raise ValueError("Truncated data chunk")
    return data


def parse_generic_response(block: bytes):
    """Parse any response message and return (name, values)"""
    if block[0] != len(block) or block[-1] != 0x7E:
        raise ValueError("Framing error")
    recv = int.from_bytes(block[-3:-1], "big")
    calc = crc16_ccitt(block[:-3])
    if recv != calc:
        raise ValueError("CRC mismatch")
    content = block[2:-3]
    if not content:
        raise ValueError("Empty response content")
    rid, pos = vlq_decode(content, 0)
    name = resp_id_to_name.get(rid, f"<RID_{rid}>")
    vals = []
    while pos < len(content):
        v, pos = vlq_decode(content, pos)
        vals.append(v)
    return name, vals


def query_dictionary(ser: serial.Serial, max_attempts=10) -> dict:
    """Query MCU for its command/response dictionary"""
    print("=== Querying MCU dictionary ===")
    comp = bytearray()
    offset = 0
    chunks = 0
    seq = 0
    retries = 0
    identify_cmd_id = 1
    request_timeout = 1.5

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    while True:
        pkt = build_block(identify_cmd_id, [offset, CHUNK_SIZE], seq)
        ser.write(pkt)
        ack_count = 0
        other_count = 0
        chunk = None
        last_error = None
        last_ack_seq = None
        request_deadline = time.monotonic() + request_timeout

        while time.monotonic() < request_deadline:
            remaining = max(0.05, request_deadline - time.monotonic())
            try:
                blk = read_block(ser, timeout=remaining)
            except TimeoutError as exc:
                last_error = exc
                break

            # Skip ACK/NAK messages, but don't loop forever without progress.
            # The MCU's next_sequence value is echoed in every message it
            # sends (including ACK/NAK), so use it to resync our sequence
            # counter if it doesn't match what the MCU expects (e.g. a stale
            # session left the MCU's sequence counter advanced).
            if blk[0] == 5:
                ack_count += 1
                last_ack_seq = blk[1] & 0x0F
                continue

            try:
                chunk = parse_identify_response(blk)
                retries = 0
                break
            except ValueError as exc:
                last_error = exc
                # Ignore unrelated traffic, but still bound the wait for a real
                # identify_response so startup can't hang silently.
                if len(blk) >= 5:
                    try:
                        content = blk[2:-3]
                        resp_id, _ = vlq_decode(content, 0)
                        if resp_id not in (0, 1):
                            other_count += 1
                            continue
                    except Exception:
                        pass
                print(f"WARNING: Parse error (attempt {retries+1}/{max_attempts}): {exc}")
                print(f"   Block: {blk.hex()}")
                break

        if chunk is None:
            retries += 1
            if ack_count or other_count:
                detail = []
                if ack_count:
                    detail.append(f"{ack_count} ACK")
                if other_count:
                    detail.append(f"{other_count} unrelated response")
                suffix = ", saw " + " and ".join(detail)
            elif last_error is not None:
                suffix = f", last error: {last_error}"
            else:
                suffix = ""
            print(f"WARNING: Identify attempt {retries}/{max_attempts} made no progress{suffix}")
            if retries >= max_attempts:
                raise TimeoutError(
                    "Failed to get MCU dictionary. The MCU may still be configured, "
                    "busy, or not responding to identify on this port."
                )
            if last_ack_seq is not None and last_ack_seq != seq:
                print(f"   Resyncing sequence counter to MCU-reported value {last_ack_seq}")
                seq = last_ack_seq
            ser.reset_input_buffer()
            time.sleep(0.1)
            continue

        comp.extend(chunk)
        offset += len(chunk)
        chunks += 1
        sys.stdout.write(f"\rDictionary: {offset} bytes ({chunks} chunks)")
        sys.stdout.flush()
        seq = (seq + 1) & 0x0F

        if len(chunk) < CHUNK_SIZE:
            break

    print("\n==================================\n")

    plain = zlib.decompress(bytes(comp))
    d = json.loads(plain)

    version_label = d["version"].strip('"')
    print(f"dict_v{version_label} loaded\n")

    global cmd_id_map, resp_id_to_name
    cmd_id_map = {k.split()[0]: v for k, v in d["commands"].items()}
    resp_id_to_name = {v: k.split()[0] for k, v in d["responses"].items()}

    # Return both dictionary and final sequence number for proper sync
    return d, seq, plain


def save_dictionary(dict_plain: bytes, path: str):
    dict_path = Path(path)
    dict_path.write_text(dict_plain.decode("utf-8"))
    print(f"Dictionary saved to {dict_path}\n")


def reset_serial_tty_state(port: str):
    try:
        proc = subprocess.run(
            ["stty", "-F", port, "sane"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
        if proc.returncode != 0:
            detail = proc.stderr.strip() or f"exit status {proc.returncode}"
            print(f"WARNING: 'stty -F {port} sane' failed: {detail}")
    except Exception as exc:
        print(f"WARNING: Failed to run 'stty -F {port} sane': {exc}")


def open_serial_port(port: str, baud: int, announce=True) -> serial.Serial:
    reset_serial_tty_state(port)
    if announce:
        print(f"Opening {port} at {baud} baud...\n")
    return serial.Serial(port, baud, timeout=TIMEOUT, exclusive=True)


def connect_and_query(port: str, baud: int, save_dict_path: str = None,
                      announce=True, attempts=10, retry_delay=0.5):
    last_exc = None
    for attempt in range(1, attempts + 1):
        ser = None
        try:
            ser = open_serial_port(port, baud, announce=announce and attempt == 1)
            dictionary, seq, dict_plain = query_dictionary(ser)
            if save_dict_path:
                save_dictionary(dict_plain, save_dict_path)
            return ser, dictionary, seq, dict_plain
        except Exception as exc:
            last_exc = exc
            if ser is not None:
                ser.close()
            if attempt >= attempts:
                break
            print(f"Reconnect attempt {attempt}/{attempts} failed: {exc}")
            time.sleep(retry_delay)
    raise RuntimeError(f"Unable to establish MCU session: {last_exc}") from last_exc


def parse_version_string(version_str: str):
    version_label = version_str.strip('"').strip()
    version_number = version_label.lstrip("v")
    parts = version_number.split(".")
    if len(parts) != 3:
        raise ValueError(f"Invalid version string: {version_str!r}")
    return version_number, [int(p) for p in parts]


def decode_remote_info(vals):
    if not vals or len(vals) < 6:
        return None
    return {
        "oid": vals[0],
        "flag": vals[1],
        "crc32": vals[2],
        "version_major": vals[3],
        "version_minor": vals[4],
        "version_patch": vals[5],
        "version": f"{vals[3]}.{vals[4]}.{vals[5]}",
    }


def print_remote_info(prefix: str, info):
    print(f"{prefix}:")
    print(
        f"   flag={info['flag']} version={info['version']} "
        f"CRC32=0x{info['crc32']:08X}\n"
    )


def query_config_state(ser: serial.Serial, seq: int):
    if "get_config" not in cmd_id_map:
        print("WARNING: MCU dictionary has no get_config command\n")
        return seq, None

    pkt = build_block(cmd_id_map["get_config"], [], seq)
    ser.write(pkt)
    ser.flush()
    seq = (seq + 1) & 0x0F

    end_time = time.time() + 2.0
    while time.time() < end_time:
        try:
            blk = read_block(ser, timeout=max(0.1, end_time - time.time()))
        except TimeoutError:
            break

        if blk[0] == 5:
            continue

        try:
            name, vals = parse_generic_response(blk)
        except Exception as exc:
            print(f"Parse error while waiting for config: {exc}")
            continue

        if name == "stats":
            continue
        if name == "config":
            if len(vals) < 4:
                raise ValueError(f"Short config response: {vals}")
            return seq, {
                "is_config": vals[0],
                "crc": vals[1],
                "is_shutdown": vals[2],
                "move_count": vals[3],
            }

    print("WARNING: No config response\n")
    return seq, None


def print_config_state(config_state):
    if not config_state:
        return
    print("MCU config state:")
    print(
        f"   is_config={config_state['is_config']} "
        f"is_shutdown={config_state['is_shutdown']} "
        f"crc=0x{config_state['crc']:08X} "
        f"move_count={config_state['move_count']}\n"
    )


def send_simple_command(ser: serial.Serial, cmd_name: str, params, seq: int):
    cid = cmd_id_map.get(cmd_name)
    if cid is None:
        return seq, False
    pkt = build_block(cid, params, seq)
    ser.write(pkt)
    ser.flush()
    return (seq + 1) & 0x0F, True


def send_klipper_config_sequence(ser: serial.Serial, oid: int, seq: int = 0):
    """Send Klipper configuration sequence"""
    max_oids = oid + 1
    seq_start = seq

    # Clear any pending data
    ser.reset_input_buffer()
    
    for cmd_name, params in (
        ("allocate_oids", [max_oids]),
        ("config_ota", [oid]),
    ):
        cid = cmd_id_map[cmd_name]
        pkt = build_block(cid, params, seq)
        print(f"  -> {cmd_name} (cmd_id={cid}, params={params})")
        ser.write(pkt)
        seq = (seq + 1) & 0x0F
        time.sleep(0.05)

    finalize_crc = zlib.crc32(f"allocate_oids count={max_oids}\nconfig_ota oid={oid}".encode("utf-8")) & 0xFFFFFFFF
    pkt = build_block(cmd_id_map["finalize_config"], [finalize_crc], seq)
    print(f"  -> finalize_config (crc=0x{finalize_crc:08X})")
    ser.write(pkt)
    seq = (seq + 1) & 0x0F
    time.sleep(0.3)

    print("\n  Verifying config via get_config...")
    seq, config_state = query_config_state(ser, seq)
    if config_state:
        print(
            f"  Config state: is_config={config_state['is_config']} "
            f"is_shutdown={config_state['is_shutdown']} "
            f"crc=0x{config_state['crc']:08X}"
        )
    print("Config sent\n")
    return seq, seq_start


def query_remote_version(ser: serial.Serial, oid: int, seq: int):
    """Query MCU's current OTA info"""
    if "query_ota_local_info" not in cmd_id_map:
        return seq, None

    cid = cmd_id_map["query_ota_local_info"]
    pkt = build_block(cid, [oid], seq)
    ser.write(pkt)
    seq = (seq + 1) & 0x0F

    end_time = time.time() + 2.0
    while time.time() < end_time:
        try:
            blk = read_block(ser, timeout=max(0.1, end_time - time.time()))
        except TimeoutError:
            break

        if blk[0] == 5:
            continue

        try:
            name, vals = parse_generic_response(blk)
        except Exception as exc:
            print(f"Parse error: {exc}")
            continue

        if name == "ota_local_info":
            return seq, vals

    print("WARNING: No ota_local_info response\n")
    return seq, None


def derive_version_from_firmware(fw_path: str):
    """
    Try to determine the firmware version without requiring --version.

    Preference order:
    1. A sibling command dictionary (klipper.dict, or <name>.dict next to
       the .bin) written by scripts/buildcommands.py at build time - this
       is the authoritative source since it is generated directly from
       CONFIG_FIRMWARE_VERSION.
    2. The version encoded in the build.sh output filename, e.g.
       firmware_v2.0.10_20260703.bin or firmware_v2.0.10_OS_20260703.bin.

    Returns the version string (e.g. "2.0.10") or None if it can't be
    determined either way.
    """
    fw = Path(fw_path)

    for candidate in (fw.with_suffix(".dict"), fw.parent / "klipper.dict"):
        if candidate.exists():
            try:
                d = json.loads(candidate.read_text())
                version = str(d["version"]).strip('"').lstrip("v")
                print(f"Version from {candidate.name}: {version}")
                return version
            except Exception as exc:
                print(f"WARNING: Failed to read version from {candidate}: {exc}")

    match = re.search(r"_v(\d+\.\d+\.\d+)_", fw.name)
    if match:
        version = match.group(1)
        print(f"Version from firmware filename: {version}")
        return version

    return None


def prepare_firmware_with_crc(fw_path: str):
    """
    Read firmware and extract/verify CRC.
    CRITICAL: File format is [data][8 ASCII hex][newline]
    CRC is calculated over data only (excluding last 9 bytes).
    """
    with open(fw_path, "rb") as f:
        fw_binary = f.read()

    print("\n=== Preparing Firmware ===")
    print(f"File size: {len(fw_binary)} bytes")

    # File ends with 8 ASCII hex digits + newline (9 bytes total)
    if len(fw_binary) < 9:
        raise ValueError("Firmware too small")

    # Extract CRC (last 8 bytes before the newline)
    crc_ascii = fw_binary[-9:-1]
    try:
        crc_from_fw = int(crc_ascii.decode("ascii"), 16)
    except (UnicodeDecodeError, ValueError) as e:
        raise ValueError(f"Invalid CRC format: {crc_ascii!r}") from e

    print(f"CRC in firmware: 0x{crc_from_fw:08X}")

    # Calculate CRC over everything except last 9 bytes (8 hex + newline)
    # This matches Go: byteArr[:len(byteArr)-9]
    fw_without_crc = fw_binary[:-9]
    crc_calculated = zlib.crc32(fw_without_crc) & 0xFFFFFFFF
    print(f"CRC calculated: 0x{crc_calculated:08X}")

    if crc_calculated != crc_from_fw:
        raise ValueError(f"CRC mismatch: file=0x{crc_from_fw:08X} calc=0x{crc_calculated:08X}")

    print("CRC verified\n")
    return fw_binary, crc_from_fw


def ensure_flashable_state(ser: serial.Serial, seq: int, oid: int,
                           port: str, baud: int, save_dict_path: str = None):
    print("=== Startup Preflight ===")
    seq, config_state = query_config_state(ser, seq)
    print_config_state(config_state)

    if config_state and config_state["is_shutdown"]:
        if "config_reset" in cmd_id_map:
            print("MCU is in shutdown. Trying config_reset...\n")
            seq, _ = send_simple_command(ser, "config_reset", [], seq)
            time.sleep(0.2)
            seq, config_state = query_config_state(ser, seq)
            print_config_state(config_state)
        if config_state and config_state["is_shutdown"] and "reset" in cmd_id_map:
            print("MCU is still in shutdown. Trying reset and reconnect...\n")
            seq, _ = send_simple_command(ser, "reset", [], seq)
            time.sleep(0.2)
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(1.0)
            ser, _, seq, _ = connect_and_query(
                port, baud, save_dict_path=save_dict_path, announce=False
            )
            print("Reconnected after startup reset\n")
            seq, config_state = query_config_state(ser, seq)
            print_config_state(config_state)
        if config_state and config_state["is_shutdown"]:
            raise RuntimeError(
                "MCU is in shutdown and could not be recovered automatically. "
                "Reset the MCU manually and rerun the script."
            )

    if config_state and config_state["is_config"]:
        if "reset" not in cmd_id_map:
            raise RuntimeError(
                "MCU is already configured and has no reset command. "
                "Stop Klipper and reset the MCU manually, then rerun the script."
            )
        print("MCU is already configured. Trying reset and reconnect...\n")
        seq, _ = send_simple_command(ser, "reset", [], seq)
        time.sleep(0.2)
        try:
            ser.close()
        except Exception:
            pass
        time.sleep(1.0)
        ser, _, seq, _ = connect_and_query(
            port, baud, save_dict_path=save_dict_path, announce=False
        )
        print("Reconnected after startup reset\n")
        seq, config_state = query_config_state(ser, seq)
        print_config_state(config_state)
        if config_state and config_state["is_config"]:
            raise RuntimeError(
                "MCU is still configured after automatic reset. "
                "Stop Klipper and reset the MCU manually, then rerun the script."
            )
        if config_state and config_state["is_shutdown"]:
            raise RuntimeError(
                "MCU entered shutdown after automatic reset. "
                "Reset the MCU manually and rerun the script."
            )

    seq, _ = send_klipper_config_sequence(ser, oid, seq)
    seq, remote_info = query_remote_version(ser, oid, seq)
    remote_state = decode_remote_info(remote_info)
    if remote_state:
        print_remote_info("MCU OTA state", remote_state)
    return ser, seq, remote_state


class OTAState:
    """State machine matching Go implementation"""
    def __init__(self, total_size: int, oid: int):
        self.state = "standby"
        self.progress = 0.0
        self.total_size = total_size
        self.oid = oid
        self.error_code = 0

    def set_state(self, new_state: str, offset: int = None):
        self.state = new_state
        if offset is not None and self.total_size:
            frac = max(0.0, min(1.0, offset / self.total_size))
            self.progress = round(frac, 2)


def ota_update_fixed(ser: serial.Serial, fw_binary: bytes, fw_crc32: int,
                     version_str: str, oid: int, seq: int):
    """
    OTA update implementation matching Go extras_ota.go algorithm.
    
    Key behaviors from Go code:
    - Send ota_start with CRC and version
    - Send ota_erase with is_transfer=1 to trigger transfer requests
    - Respond to ota_transfer requests with chunks
    - Strip trailing newline (0x0A) when at EOF
    - Send empty data response when offset >= file size
    - Wait for ota_status FINISH (0x20)
    - Do not reset or reconnect automatically
    """
    print("=== Starting OTA ===")
    print(f"Version: {version_str}")
    print(f"CRC32: 0x{fw_crc32:08X}")
    print(f"Size: {len(fw_binary)} bytes\n")

    # Parse version
    _, version_fields = parse_version_string(version_str)

    ota_state = OTAState(len(fw_binary), oid)
    ota_state.set_state("starting")

    # Step 1: Send ota_start (Go line 294)
    print("Sending ota_start...")
    pkt = build_block(
        cmd_id_map["ota_start"],
        [oid, fw_crc32, version_fields[0], version_fields[1], version_fields[2]],
        seq,
    )
    ser.write(pkt)
    ser.flush()
    seq = (seq + 1) & 0x0F
    time.sleep(0.1)

    # Step 2: Send ota_erase with is_transfer=1 (Go line 300)
    print("Sending ota_erase...")
    pkt = build_block(cmd_id_map["ota_erase"], [oid, 0, 1], seq)
    ser.write(pkt)
    ser.flush()
    seq = (seq + 1) & 0x0F
    ota_state.set_state("erasing")
    time.sleep(0.2)

    transfer_cid = cmd_id_map["ota_transfer_response"]
    bytes_transferred = 0
    last_progress_pct = None

    def update_progress(offset: int):
        nonlocal last_progress_pct
        progress_pct = int((offset / len(fw_binary)) * 100) if fw_binary else 0
        if progress_pct != last_progress_pct:
            sys.stdout.write(f"\rTransfer: {progress_pct:3d}% ({offset}/{len(fw_binary)})")
            sys.stdout.flush()
            last_progress_pct = progress_pct

    # Main loop matching Go lines 304-323
    finish_wait_count = 0
    max_finish_wait = 20  # Wait up to 20 timeouts (20 seconds) after transfer_finish
    
    while True:
        try:
            # Use longer timeout after transfer completes (MCU doing CRC)
            timeout = 5.0 if ota_state.state == "transfer_finish" else 10.0
            blk = read_block(ser, timeout)
        except TimeoutError:
            if ota_state.state in ("finished", "restarted"):
                break
            # After transfer finishes, wait a bit for FINISH status
            if ota_state.state == "transfer_finish":
                finish_wait_count += 1
                if finish_wait_count >= max_finish_wait:
                    print("\nWARNING: Timeout waiting for FINISH status after transfer complete")
                    print("    Transfer may have succeeded - check MCU status")
                    break
                print(f"\nWaiting for FINISH status... ({finish_wait_count}/{max_finish_wait})")
            else:
                print(f"\nWARNING: Read timeout (state={ota_state.state}), continuing...")
            continue
        except serial.SerialException as exc:
            # MCU likely restarted after finishing OTA
            if ota_state.state in ("finished", "restarted", "transfer_finish"):
                print("\nSerial disconnected")
                break
            print(f"\nERROR: Serial error: {exc}")
            raise

        # Skip ACK packets
        if blk[0] == 5:
            continue

        try:
            name, vals = parse_generic_response(blk)
            if name == "stats":
                # Don't spam output with stats messages
                continue
        except Exception as exc:
            print(f"\n[ERR] Parse error: {exc}, block: {blk.hex()}")
            continue

        # Handle ota_transfer request (Go _handle_ota_transfer)
        if name == "ota_transfer":
            req_oid, req_offset, req_count = vals

            # Read chunk from firmware
            data_end = min(req_offset + req_count, len(fw_binary))
            raw_chunk = fw_binary[req_offset:data_end]
            read_len = len(raw_chunk)
            new_offset = req_offset + read_len

            # CRITICAL: Send the chunk AS-IS, including any newline
            # The MCU will strip the newline during writing (otacmd.c line 166)
            # This matches Go code behavior (extras_ota.go line 181-183)
            
            # If no data to send (EOF reached), send empty response to trigger CRC check
            if not raw_chunk:
                update_progress(req_offset)
                print(f"\nEOF reached at offset {req_offset}, sending empty data for CRC check")
                pkt = build_block(transfer_cid, [oid, req_offset, b""], seq)
                ser.write(pkt)
                ser.flush()
                seq = (seq + 1) & 0x0F
                ota_state.set_state("transfer_finish", bytes_transferred)
                ota_state.progress = 1.0
                # Give MCU more time to calculate CRC (can take several seconds)
                time.sleep(1.0)
                continue

            # Send chunk (Go lines 170-171)
            pkt = build_block(transfer_cid, [oid, new_offset, raw_chunk], seq)
            ser.write(pkt)
            ser.flush()
            seq = (seq + 1) & 0x0F

            bytes_transferred = new_offset
            ota_state.set_state("writing", new_offset)
            update_progress(new_offset)

        # Handle ota_status (Go _handle_ota_status)
        elif name == "ota_status":
            req_oid, offset, status, err_code = vals

            status_map = {
                0x01: "START",
                0x02: "ERASING",
                0x04: "ERASED",
                0x08: "WRITING",
                0x10: "WRITTEN",
                0x20: "FINISH",
                0x40: "ERROR",
            }
            status_str = status_map.get(status, f"0x{status:02X}")
            print(f"\nOTA Status: {status_str} (offset={offset}, err_code={err_code})")

            # Map status to state (Go lines 198-232)
            if status == 0x01:  # OTA_UPGRADING_START
                ota_state.set_state("start")
            elif status == 0x02:  # OTA_UPGRADING_ERASING
                ota_state.set_state("erasing")
            elif status == 0x04:  # OTA_UPGRADING_ERASED
                ota_state.set_state("erase finish")
            elif status == 0x08:  # OTA_UPGRADING_WRITING
                ota_state.set_state("writing", offset)
            elif status == 0x10:  # OTA_UPGRADING_WRITTEN
                ota_state.set_state("write finish", offset)
            elif status == 0x20:  # OTA_UPGRADING_FINISH
                ota_state.set_state("finished")
                print(f"\n{'='*60}")
                print("OTA complete - firmware staged successfully")
                print(f"{'='*60}")
            elif status == 0x40:  # OTA_UPGRADING_ERROR
                ota_state.error_code = err_code
                error_names = {
                    20: "OD_CHECK_ERR",
                    21: "OD_WRITE_ERR",
                    22: "WRITE_ERR",
                    23: "ERASE_ERR",
                    24: "CRC_ERR"
                }
                err_str = error_names.get(err_code, f"UNKNOWN_{err_code}")
                ota_state.set_state(f"error_{err_str}")
                raise RuntimeError(f"OTA failed with error: {err_str} (code {err_code})")

        # Handle CRC check message (critical - shows if CRC matched)
        elif name == "ota_crc_check":
            if len(vals) >= 7:
                _, flag, v1, v2, v3, crc_fw, crc_calc = vals[:7]
                print(f"\n{'='*60}")
                print("MCU CRC check:")
                print(f"   Version: {v1}.{v2}.{v3}")
                print(f"   CRC from FW: 0x{crc_fw:08X}")
                print(f"   CRC calculated: 0x{crc_calc:08X}")
                if crc_fw == crc_calc:
                    print("   CRC match - OTA data will be saved")
                else:
                    print("   CRC mismatch - OTA will fail")
                print(f"{'='*60}")
            else:
                print(f"\nMCU CRC check: {vals}")

        # Handle save OTA data (critical - confirms OTA data was saved)
        elif name == "ota_save_otadata":
            if len(vals) >= 8:
                _, flag, v1, v2, v3, crc, _, offset = vals
                print(f"\n{'='*60}")
                print("MCU saved OTA data successfully:")
                print(f"   app_flag: {flag} (2=APP_DOWNLOAD, will update on reboot)")
                print(f"   Version: {v1}.{v2}.{v3}")
                print(f"   CRC32: 0x{crc:08X}")
                print(f"   Offset: {offset} bytes (0x{offset:X})")
                print(f"{'='*60}")
            else:
                print(f"\nMCU saved OTA data: {vals}")

        # Handle MCU restart message (indicates successful OTA)
        elif name == "starting":
            if ota_state.state == "transfer_finish":
                print(f"\n{'='*60}")
                print("MCU restarted during OTA session")
                print(f"{'='*60}")
                ota_state.set_state("restarted")

        # Exit condition (Go lines 312-322)
        if ota_state.state in ("finished", "restarted"):
            print(f"\nState is '{ota_state.state}', exiting loop")
            break
        if ota_state.state.startswith("error"):
            raise RuntimeError(f"OTA error: {ota_state.state}")

    if last_progress_pct is not None:
        print()
    print(f"\n{'='*60}")
    print("OTA update complete")
    print(f"   Final state: {ota_state.state}")
    print(f"   Final progress: {ota_state.progress * 100:.1f}%")
    print(f"   Bytes transferred: {bytes_transferred}/{len(fw_binary)}")
    print(f"{'='*60}\n")

    return seq, ota_state.state


def print_disclaimer_and_confirm():
    """Print a big risk disclaimer and require the user to explicitly
    type 'Yes' to continue. Exits the script (no changes made) otherwise."""
    banner = "!" * 78
    print("\n" + banner)
    print("!!  WARNING - MCU / PRINTER FIRMWARE OTA FLASH - READ BEFORE PROCEEDING  !!")
    print(banner)
    print("""
  This tool flashes new firmware directly onto the printer's MCU via the
  Anycubic serial/OTA MCU protocol.

  THIS CAN PERMANENTLY BRICK YOUR MCU / PRINTER MAINBOARD.

  If the flash fails partway through, the new firmware build is incompatible
  with this board, or the new firmware crashes/hangs the MCU, THE BOARD MAY
  NO LONGER BOOT and could require an SWD/JTAG hardware debugger (and
  opening/disassembling the printer) to recover it.

  YOU are solely responsible for verifying the firmware you are about to
  flash and that you understand what this script does
  before running it.

  USE THIS SCRIPT ENTIRELY AT YOUR OWN RISK. If it breaks your MCU or your
  printer, that is your responsibility - don't cry about it, it's on you.
""")
    print(banner + "\n")

    try:
        answer = input("Type 'Yes' (exactly) to acknowledge and continue (anything else aborts): ")
    except (EOFError, KeyboardInterrupt):
        answer = ""

    if answer.strip() != "Yes":
        print("\nDisclaimer not acknowledged. Aborting - no changes made.")
        sys.exit(1)
    print("")


def main():
    parser = argparse.ArgumentParser(
        description="Anycubic Kobra MCU OTA updater",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "\nExample:\n"
            "  python3 mcu_ota_flasher.py --port /dev/ttyGS1 "
            "--firmware out/firmware_v2.0.10_20260706.bin"
        )
    )
    parser.add_argument("--port", required=True, help="Serial port")
    parser.add_argument("--baud", type=int, default=576000, help="Baud rate")
    parser.add_argument("--firmware", required=True, help="Firmware binary (.bin)")
    parser.add_argument("--version", metavar="X.Y.Z", default=None,
                       help="Firmware version, e.g. 2.0.10 (optional). If "
                            "omitted, it is auto-detected from a sibling "
                            "klipper.dict/.dict file next to --firmware, or "
                            "from a _vX.Y.Z_ pattern in its filename.")
    parser.add_argument("--save-dict", metavar="FILE", help="Save MCU dictionary to file")
    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(1)
    args = parser.parse_args()

    print_disclaimer_and_confirm()

    ser = None
    try:
        fw_binary, fw_crc32 = prepare_firmware_with_crc(args.firmware)

        version_str = args.version
        if version_str is None:
            version_str = derive_version_from_firmware(args.firmware)
            if version_str is None:
                raise RuntimeError(
                    "Could not auto-detect firmware version (no sibling "
                    "klipper.dict/.dict file and no _vX.Y.Z_ pattern in the "
                    "filename). Pass --version explicitly."
                )

        # Query dictionary - returns final seq for sync
        ser, dictionary, seq, dict_plain = connect_and_query(
            args.port, args.baud, save_dict_path=args.save_dict
        )
        print(f"Continuing with seq={seq} after dictionary query")

        ser, seq, remote_state = ensure_flashable_state(
            ser, seq, OTA_OID, args.port, args.baud, save_dict_path=args.save_dict
        )

        # Perform OTA update
        seq, final_state = ota_update_fixed(
            ser,
            fw_binary,
            fw_crc32,
            version_str,
            OTA_OID,
            seq,
        )

        print("OTA script complete.\n")
        print("Reset the MCU manually to apply the staged firmware.\n")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(1)
    except Exception as exc:
        print(f"\nERROR: {exc}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
