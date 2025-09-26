#!/usr/bin/env python3
"""
serial_to_vcan.py
-----------------
Reads DM_IMU-style frames from a serial device (default /dev/gimbal)
parses roll/pitch/yaw and sends a CAN frame with quaternion payload to vcan0.

Usage:
  pip3 install pyserial
  sudo modprobe vcan
  sudo ip link add dev vcan0 type vcan
  sudo ip link set up vcan0
  python3 tools/serial_to_vcan.py --port /dev/gimbal --baud 921600 --can vcan0

The script sends CAN frames with can_id 0x01 and 8-byte payload:
  data[0..1] = int16_t(x * 1e4) big-endian
  data[2..3] = int16_t(y * 1e4)
  data[4..5] = int16_t(z * 1e4)
  data[6..7] = int16_t(w * 1e4)

This matches how CBoard::callback unpacks quaternion frames in io/cboard.cpp

Note: The DM_IMU serial frame parser here implements a minimal compatible
subset to extract roll/pitch/yaw floats, based on io/dm_imu/dm_imu.cpp.
"""

import argparse
import math
import struct
import sys
import time

try:
    import serial
except Exception as e:
    print("Please install pyserial: pip3 install pyserial")
    raise

import socket
import os

# helper to send raw CAN frames using PF_CAN sockets (no python-can dependency)
class RawCanSocket:
    def __init__(self, ifname='vcan0'):
        # create PF_CAN RAW socket
        self.ifname = ifname
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)

        # Prefer the portable socket.if_nametoindex to get the interface index.
        try:
            ifindex = socket.if_nametoindex(ifname)
        except OSError as e:
            # provide a clearer error for missing or down interfaces
            raise RuntimeError(f"Error getting interface index for {ifname}: {e}")

        # bind the socket to the interface name (socket lib accepts a tuple (ifname,))
        try:
            self.sock.bind((ifname,))
        except OSError as e:
            raise RuntimeError(f"Error binding CAN socket to {ifname}: {e}")

    def send(self, can_id, data_bytes):
        # can_frame struct: can_id (unsigned int), can_dlc (unsigned char), __pad etc, data (8)
        # We'll pack as: <I B 3x 8s (total 16+?) but use low-level send
        if len(data_bytes) != 8:
            raise ValueError('data_bytes must be length 8')
        can_id_packed = struct.pack('<I', can_id)
        can_dlc = struct.pack('B', 8)
        padding = b'\x00' * 3
        payload = data_bytes
        frame = can_id_packed + can_dlc + padding + payload
        # linux requires sending sizeof(can_frame)=16? but SOCK_RAW allows this packing
        self.sock.send(frame)


# DM_IMU frame minimal parser
class DMParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, b: bytes):
        self.buf += b
        # look for header pattern: 0x55 .. flag1 0xAA etc; based on dm_imu.cpp it reads 4 bytes then rest 57-4
        # We'll search for known headers in buffer and try to parse frames.
        frames = []
        while True:
            if len(self.buf) < 4:
                break

            # If buffer starts with alternative header 0xFF 0x01 0x00 0x00, parse that first.
            if self.buf.startswith(b"\xff\x01\x00\x00"):
                # need at least 4 + 12 bytes for three floats and a terminator 0x0d
                if len(self.buf) < 4 + 12 + 1:
                    break
                # find terminator 0x0d after payload
                try:
                    term = self.buf.index(0x0d, 4 + 12)
                except ValueError:
                    # packet not complete yet
                    break
                chunk = self.buf[:term+1]
                # extract three little-endian floats at offsets 4,8,12
                try:
                    roll = struct.unpack_from('<f', chunk, 4)[0]
                    pitch = struct.unpack_from('<f', chunk, 8)[0]
                    yaw = struct.unpack_from('<f', chunk, 12)[0]
                except struct.error:
                    # malformed, drop one byte and continue
                    del self.buf[0]
                    continue
                frames.append((roll, pitch, yaw))
                # remove processed packet
                del self.buf[:term+1]
                continue

            # Otherwise try the original 0x55-based DM_IMU packets (57 bytes)
            # find 0x55 header
            try:
                idx = self.buf.index(0x55)
            except ValueError:
                # no 0x55 header present; drop buffer (we already handled ff headers above)
                self.buf.clear()
                break
            if idx > 0:
                del self.buf[:idx]
            if len(self.buf) < 57:
                break
            chunk = self.buf[:57]
            # minimal checks: FrameHeader1==0x55, flag1==0xAA, slave_id1==0x01, reg_acc==0x01
            FrameHeader1 = chunk[0]
            flag1 = chunk[1]
            slave_id1 = chunk[2]
            reg_acc = chunk[3]
            if not (FrameHeader1 == 0x55 and flag1 == 0xAA and slave_id1 == 0x01 and reg_acc == 0x01):
                # drop first byte and continue
                del self.buf[0]
                continue
            try:
                accx = struct.unpack_from('<f', chunk, 4)[0]
                accy = struct.unpack_from('<f', chunk, 8)[0]
                accz = struct.unpack_from('<f', chunk, 12)[0]
                gyrox = struct.unpack_from('<f', chunk, 16)[0]
                gyroy = struct.unpack_from('<f', chunk, 20)[0]
                gyroz = struct.unpack_from('<f', chunk, 24)[0]
                roll = struct.unpack_from('<f', chunk, 28)[0]
                pitch = struct.unpack_from('<f', chunk, 32)[0]
                yaw = struct.unpack_from('<f', chunk, 36)[0]
            except struct.error:
                del self.buf[0]
                continue
            frames.append((roll, pitch, yaw))
            del self.buf[:57]
        return frames


def euler_to_quat(roll_deg, pitch_deg, yaw_deg):
    # dm_imu.cpp: yaw(Z), pitch(Y), roll(X) in degrees -> quaternion via Eigen AngleAxisd order Z, Y, X
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    # construct quaternion equivalent
    # q = Rz(yaw) * Ry(pitch) * Rx(roll)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    # following quaternion multiplication order (w, x, y, z)
    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr
    # normalize
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n == 0:
        return (1.0, 0.0, 0.0, 0.0)
    return (w/n, x/n, y/n, z/n)


def pack_quat_to_can_bytes(q):
    # CBoard::callback expects int16 for x,y,z,w with /1e4 (frame.data[0..7]: xH,xL,yH,yL,...)
    # Note: CBoard unpacks as int16_t(frame.data[0]<<8 | frame.data[1]) / 1e4 => big-endian
    # So we should pack each component as int16 big-endian of (component * 1e4)
    w, x, y, z = q
    # CBoard uses order: x,y,z,w when parsing? In CBoard::callback it does:
    # auto x = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e4; ... w = (int16_t)(frame.data[6] << 8 | frame.data[7]) / 1e4;
    # So the order is x,y,z,w
    components = [x, y, z, w]
    out = bytearray(8)
    for i, comp in enumerate(components):
        val = int(round(comp * 1e4))
        if val < -32768: val = -32768
        if val > 32767: val = 32767
        hi = (val >> 8) & 0xff
        lo = val & 0xff
        out[i*2] = hi
        out[i*2+1] = lo
    return bytes(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/gimbal', help='serial port to read DM IMU from (default /dev/gimbal)')
    parser.add_argument('--baud', type=int, default=921600)
    parser.add_argument('--can', default='vcan0', help='socketcan interface to send (default vcan0)')
    parser.add_argument('--canid', type=lambda x: int(x,0), default=0x01, help='can id for quaternion (default 0x01)')
    parser.add_argument('--debug', action='store_true', help='print raw serial bytes for debugging')
    args = parser.parse_args()

    # The device observed on your vehicle maps axes differently from our original assumption.
    # Per your note, we hardcode the mapping here (no YAML):
    #   original (serial) tuple: (roll, pitch, yaw) == (orig_r, orig_p, orig_y)
    #   desired mapping (real):
    #     real_roll  = orig_y
    #     real_pitch = orig_r
    #     real_yaw   = -orig_p
    # This corresponds to map='yrp' and flip='++-'. We override any CLI map/flip to these values.
    map_str = 'yrp'
    flip_str = '+-+'
    print(f"[serial_to_vcan] using hardcoded mapping map={map_str} flip={flip_str}")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except Exception as e:
        print(f"Failed to open serial port {args.port}: {e}")
        sys.exit(1)

    try:
        can_sock = RawCanSocket(args.can)
    except Exception as e:
        print(f"Failed to open CAN socket on {args.can}: {e}")
        sys.exit(1)

    parser = DMParser()
    print("serial_to_vcan started, reading %s -> %s (canid=0x%02x)" % (args.port, args.can, args.canid))

    try:
        while True:
            data = ser.read(256)
            if data:
                if args.debug:
                    print(f"[DEBUG] read {len(data)} bytes: {data.hex()}")
                frames = parser.feed(data)
                for roll, pitch, yaw in frames:
                    # original values from serial: (roll, pitch, yaw)
                    # apply the hardcoded mapping:
                    #   mapped_roll  = orig_yaw
                    #   mapped_pitch = orig_roll
                    #   mapped_yaw   = -orig_pitch
                    m_roll = yaw
                    m_pitch = roll
                    m_yaw = -pitch
                    q = euler_to_quat(m_roll, m_pitch, m_yaw)
                    can_bytes = pack_quat_to_can_bytes(q)
                    try:
                        can_sock.send(args.canid, can_bytes)
                        if args.debug:
                            print(f"sent can quat: orig r={roll:.3f} p={pitch:.3f} y={yaw:.3f} -> mapped r={m_roll:.3f} p={m_pitch:.3f} y={m_yaw:.3f} q={q}")
                        else:
                            print(f"sent can quat (canid=0x{args.canid:02x})")
                    except Exception as e:
                        print(f"Failed to send CAN frame: {e}")
            else:
                if args.debug:
                    # indicate waiting
                    print("[DEBUG] no data read")
                time.sleep(0.01)
    except KeyboardInterrupt:
        print("exiting")


if __name__ == '__main__':
    main()
