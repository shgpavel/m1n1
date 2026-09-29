# SPDX-License-Identifier: MIT
from construct import Struct, Int8ul, Int16ul, Int32sl, Int32ul, Int64ul
from subprocess import Popen, PIPE
import pathlib
import struct
import os
import sys

from ..utils import *

VirtioConfig = Struct(
    "irq" / Int32sl,
    "devid" / Int32ul,
    "feats" / Int64ul,
    "num_qus" / Int32ul,
    "data" / Int64ul,
    "data_len" / Int64ul,
    "verbose" / Int8ul,
)

class VirtioDescFlags(Register16):
    WRITE = 1
    NEXT  = 0

VirtioDesc = Struct(
    "addr" / Int64ul,
    "len" / Int32ul,
    "flags" / RegAdapter(VirtioDescFlags),
    "next" / Int16ul,
)

VirtioExcInfo = Struct(
    "devbase" / Int64ul,
    "qu" / Int16ul,
    "idx" / Int16ul,
    "pad" / Int32ul, 
    "descbase" / Int64ul,
)

class VirtioDev:
    def __init__(self):
        self.base, self.hv = None, None # assigned by HV object

    def read_buf(self, desc):
        return self.hv.iface.readmem(desc.addr, desc.len)

    def read_desc(self, ctx, idx):
        off = VirtioDesc.sizeof() * idx
        return self.hv.iface.readstruct(ctx.descbase + off, VirtioDesc)

    @property
    def config_data(self):
        return b""

    @property
    def devid(self):
        return 0

    @property
    def num_qus(self):
        return 1

    @property
    def feats(self):
        return 0

class Virtio9PTransport(VirtioDev):
    def __init__(self, tag="m1n1", root=None):
        p_stdin, self.fin = os.pipe()
        self.fout, p_stdout = os.pipe()
        if root is None:
            root = str(pathlib.Path(__file__).resolve().parents[3])
        if type(tag) is str:
            self.tag = tag.encode("ascii")
        else:
            self.tag = tag
        self.p = Popen([
            "u9fs",
            "-a", "none", # no auth
            "-n", # not a network conn
            "-u", os.getlogin(), # single user
            root,
        ], stdin=p_stdin, stdout=p_stdout, stderr=sys.stderr)

    @property
    def config_data(self):
        return struct.pack("=H", len(self.tag)) + self.tag

    @property
    def devid(self):
        return 9

    @property
    def num_qus(self):
        return 1

    @property
    def feats(self):
        return 1

    def call(self, req):
        os.write(self.fin, req)
        resp = os.read(self.fout, 4)
        length = int.from_bytes(resp, byteorder="little")
        resp += os.read(self.fout, length - 4)
        return resp

    def handle_exc(self, ctx):
        head = self.read_desc(ctx, ctx.idx)
        assert not head.flags.WRITE

        req = bytearray()

        while not head.flags.WRITE:
            req += self.read_buf(head)

            if not head.flags.NEXT:
                break
            head = self.read_desc(ctx, head.next)

        resp = self.call(bytes(req))
        resplen = len(resp)

        while len(resp):
            self.hv.iface.writemem(head.addr, resp[:head.len])
            resp = resp[head.len:]
            if not head.flags.NEXT:
                break
            head = self.read_desc(ctx, head.next)

        self.hv.p.virtio_put_buffer(ctx.devbase, ctx.qu, ctx.idx, resplen)

        return True

class VirtioBlk(VirtioDev):
    T_IN, T_OUT, T_FLUSH, T_GET_ID = 0, 1, 4, 8
    S_OK, S_IOERR, S_UNSUPP = 0, 1, 2
    F_SEG_MAX, F_RO, F_FLUSH = 1 << 2, 1 << 5, 1 << 9
    SEG_MAX = 126

    def __init__(self, path, readonly=False):
        super().__init__()
        self.path = path
        self.readonly = readonly
        self.fd = os.open(path, os.O_RDONLY if readonly else os.O_RDWR)
        self.size = os.lseek(self.fd, 0, os.SEEK_END)

    @property
    def config_data(self):
        return struct.pack("<QII", self.size // 512, 0, self.SEG_MAX)

    @property
    def devid(self):
        return 2

    @property
    def num_qus(self):
        return 1

    @property
    def feats(self):
        return self.F_SEG_MAX | self.F_FLUSH | (self.F_RO if self.readonly else 0)

    def handle_exc(self, ctx):
        chain = [self.read_desc(ctx, ctx.idx)]
        while chain[-1].flags.NEXT:
            chain.append(self.read_desc(ctx, chain[-1].next))
        hdr, data, status = chain[0], chain[1:-1], chain[-1]

        req, _, sector = struct.unpack("<IIQ", self.read_buf(hdr)[:16])
        off = sector * 512
        written = 0
        st = self.S_OK

        try:
            if req == self.T_IN:
                for d in data:
                    buf = os.pread(self.fd, d.len, off).ljust(d.len, b"\0")
                    self.hv.iface.writemem(d.addr, buf)
                    off += d.len
                    written += d.len
            elif req == self.T_OUT and not self.readonly:
                for d in data:
                    os.pwrite(self.fd, self.read_buf(d), off)
                    off += d.len
            elif req == self.T_OUT:
                st = self.S_IOERR
            elif req == self.T_FLUSH:
                os.fsync(self.fd)
            elif req == self.T_GET_ID and data:
                ident = os.path.basename(self.path).encode()[:20].ljust(20, b"\0")
                n = min(len(ident), data[0].len)
                self.hv.iface.writemem(data[0].addr, ident[:n])
                written += n
            else:
                st = self.S_UNSUPP
        except OSError:
            st = self.S_IOERR

        self.hv.iface.writemem(status.addr, bytes([st]))
        self.hv.p.virtio_put_buffer(ctx.devbase, ctx.qu, ctx.idx, written + 1)
        return True
