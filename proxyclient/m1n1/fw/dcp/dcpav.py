# SPDX-License-Identifier: GPL-2.0-or-later
from construct import *

from ...utils import *
from ..asc import StandardASC
from ..afk.epic import *

class DCPAVControllerService(EPICStandardService):
    NAME = "dcpav-controller-epic"
    SHORT = "dcpav"

    def setPower(self, power):
        self.call(8, 0x8, struct.pack("<16xI12x", power))

    def getPower(self, power):
        return struct.unpack("<16xI12x", self.call(8, 0x9, bytes(32)))

    def wakeDisplay(self):
        self.call(8, 0xa, bytes(16))

    def sleepDisplay(self):
        self.call(8, 0xb, bytes(16))

    def forceHotPlugDetect(self):
        self.call(8, 0xc, bytes(16))

    def setVirtualDeviceMode(self, mode):
        self.call(8, 0xd, struct.pack("<16xI12x", mode))

class DCPDPControllerService(EPICStandardService):
    NAME = "dcpdp-controller-epic"
    SHORT = "dcpdp"

class DCPDPTXEndpoint(EPICEndpoint):
    SHORT = "dptx"

    SERVICES = [
        DCPAVControllerService,
        DCPDPControllerService,
    ]

ATC0 = 0
ATC1 = 1
ATC2 = 2
ATC3 = 3
LPDPTX = 4
DPTX = 5

DPPHY = 0
DPIN0 = 1
DPIN1 = 2

class DCPDPTXRemotePortService(EPICStandardService):
    NAME = "dcpdptx-port-epic"
    SHORT = "port"

    APCALLS = [
        "Activate", "Deactivate", "GetMaxDriveSettings", "SetDriveSettings", "GetDriveSettings",
        "WillChangeLinkConfig", "DidChangeLinkConfig", "GetMaxLinkRate", "GetLinkRate", "SetLinkRate",
        "GetMaxLaneCount", "GetActiveLaneCount", "SetActiveLaneCount", "GetSupportsDownspread",
        "GetDownspread", "SetDownspread", "GetSupportsLaneMapping", "SetLaneMap", "GetSupportsHPD",
        "ForceHotPlugDetect", "InactiveSinkDetected", "SetDisplayHints", "DeviceNotResponding",
        "DeviceBusyTimeout", "DeviceNotStarted", "SetResourceAvailable", "GetVirtualDeviceMode",
    ]

    phy = None
    max_link_rate = 0x1e
    max_lane_count = 4

    def displayRequest(self):
        self.call(0, 6, bytes(16))

    def displayRelease(self):
        self.call(0, 7, bytes(16))

    def connectTo(self, connected, dfp, port=DPPHY, die=0, attrs=0):
        target = port | (dfp << 4) | (die << 8)
        if connected:
            target |= 0x8000
        resp = self.call(0, 11, struct.pack("<II24x", attrs, target))
        ret = struct.unpack_from("<I", resp, 16)[0]
        if ret:
            raise EPICError(f"connectTo {target:#x} failed: {ret:#x}")

    def hotPlugDetect(self, asserted):
        self.call(8, 8, struct.pack("<16xQ8x", int(asserted)))

    def apcall(self, group, cmd, data):
        name = self.APCALLS[cmd] if cmd < len(self.APCALLS) else f"#{cmd}"
        reply = bytearray(data)
        reply[0:4] = bytes(4)
        handler = getattr(self, "ap_" + name, None)
        if handler is not None:
            handler(data, reply)
        self.log(f"AP call {name}: {data.hex()} -> {reply.hex()}")
        return bytes(reply)

    def ap_Activate(self, data, reply):
        if self.phy:
            self.phy.activate()

    def ap_GetMaxDriveSettings(self, data, reply):
        reply[16:24] = struct.pack("<II", 3, 3)

    def ap_GetMaxLinkRate(self, data, reply):
        reply[16:24] = struct.pack("<Q", self.max_link_rate)

    def ap_GetLinkRate(self, data, reply):
        reply[16:24] = struct.pack("<Q", self.phy.rate if self.phy else 0)

    def ap_SetLinkRate(self, data, reply):
        if self.phy:
            self.phy.set_link_rate(data[16])

    def ap_GetMaxLaneCount(self, data, reply):
        reply[16:24] = struct.pack("<Q", self.max_lane_count)

    def ap_GetActiveLaneCount(self, data, reply):
        reply[16:24] = struct.pack("<Q", self.phy.lanes if self.phy else 0)

    def ap_SetActiveLaneCount(self, data, reply):
        if self.phy:
            self.phy.set_active_lane_count(struct.unpack_from("<I", data, 16)[0])

    def ap_GetSupportsHPD(self, data, reply):
        reply[16:24] = struct.pack("<Q", 1)

class DCPDPTXPortEndpoint(EPICEndpoint):
    SHORT = "dpport"

    SERVICES = [
        DCPDPTXRemotePortService,
        DCPDPControllerService,
    ]

class DCPDPDevice(EPICStandardService):
    NAME = "dcpav-device-epic"
    SHORT = "dpdev"

class DCPAVDeviceEndpoint(EPICEndpoint):
    SHORT = "avdev"

    SERVICES = [
        DCPDPDevice,
    ]

class DCPDPService(EPICStandardService):
    NAME = "dcpav-service-epic"
    SHORT = "dpserv"

class DCPAVServiceEndpoint(EPICEndpoint):
    SHORT = "avserv"

    SERVICES = [
        DCPDPService,
    ]

class DCPAVSimpleVideoInterface(EPICStandardService):
    NAME = "dcpav-video-interface-epic"
    SHORT = "video"

class DCPAVVideoEndpoint(EPICEndpoint):
    SHORT = "avserv"

    SERVICES = [
        DCPAVSimpleVideoInterface,
    ]
