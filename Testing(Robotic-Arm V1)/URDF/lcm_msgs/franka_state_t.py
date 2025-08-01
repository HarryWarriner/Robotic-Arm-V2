"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

class franka_state_t(object):

    __slots__ = ["position", "velocity", "gripper_open"]

    __typenames__ = ["double", "double", "boolean"]

    __dimensions__ = [[7], [7], None]

    def __init__(self):
        self.position = [ 0.0 for dim0 in range(7) ]
        """ LCM Type: double[7] """
        self.velocity = [ 0.0 for dim0 in range(7) ]
        """ LCM Type: double[7] """
        self.gripper_open = False
        """ LCM Type: boolean """

    def encode(self):
        buf = BytesIO()
        buf.write(franka_state_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack('>7d', *self.position[:7]))
        buf.write(struct.pack('>7d', *self.velocity[:7]))
        buf.write(struct.pack(">b", self.gripper_open))

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != franka_state_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return franka_state_t._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = franka_state_t()
        self.position = struct.unpack('>7d', buf.read(56))
        self.velocity = struct.unpack('>7d', buf.read(56))
        self.gripper_open = bool(struct.unpack('b', buf.read(1))[0])
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if franka_state_t in parents: return 0
        tmphash = (0x1182341a79d10183) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if franka_state_t._packed_fingerprint is None:
            franka_state_t._packed_fingerprint = struct.pack(">Q", franka_state_t._get_hash_recursive([]))
        return franka_state_t._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", franka_state_t._get_packed_fingerprint())[0]

