#!/usr/bin/env python
"""This module interfaces to the serial protocol used by the Teensy sensor hub."""
from __future__ import print_function
from collections import namedtuple
from datetime import datetime
import Queue

# pylint: disable=C0103
# pylint: disable=E1101

# Utility access functions
def get32(bytes_arr):
    """Create littleendian 32 bit from byte array"""
    val = bytes_arr[0] + (bytes_arr[1]<<8) + (bytes_arr[2]<<16) + (bytes_arr[3]<<24)
    return val

def get16(bytes_arr):
    """Create littleendian 16 bit from byte array"""
    val = bytes_arr[0] + (bytes_arr[1]<<8)
    return val

def getStr(bytes_arr):
    """Create string from ASCIIZ string buffer"""
    strng = ""
    for value in bytes_arr:
        if value != 0:
            strng = strng + chr(value)
        else:
            break
    return strng

def getMilliSeconds():
    """Get current machine time in milliseconds"""
    time = datetime.now()
    return time.second * 1000 + time.microsecond / 1000

# helper function
def enum(**enums):
    """Create enumerations"""
    return type('Enum', (), enums)

# Protocol data fields
header = namedtuple("header", "dst src cmd rsv")
pingpong = namedtuple("pingpong", "timestamp1 timestamp2")
distance = namedtuple("distance", "sensor distance when")
rotation = namedtuple("rotation", "speed direction when turn dist")
errorcount = namedtuple("errorcount", "count name")
#sonarwait  = namedtuple("pause")

def unpackPingPong(frm):
    """
    struct pingpong
    {
        struct header hdr;
        uint32_t timestamp1;  // ping sender fill in this (pong sender copies)
        uint32_t timestamp2;  // ping sets to 0 (pong sender fill this)
    } __attribute__((packed));
    """
    pp = pingpong(get32(frm), get32(frm[4:]))
    return pp

def unpackSonarStatus(frm):
    """
    struct distance
    {
        struct header hdr;
        uint8_t sensor;    // 0 .. MAX_NO_OF_SONAR-1
        uint8_t filler;    // Alignment
        uint16_t distance; // Measurements in microseconds
        uint32_t when;     // Time when data was measured
    } __attribute__((packed));
    """
    dist = distance(frm[0], get16(frm[2:])*10/57, get32(frm[4:]))
    return dist

def unpackRotation(frm):
    """
    struct rot_one
    {
        uint16_t speed;    // 0 Pulses per second (lowest possible speed is 20s for one wheel rev)
        uint8_t direction; // 2 Enumeration rotDirection is used
        uint8_t reserved;  // 3 Filler for alignment
        uint32_t when;     // 4 Timestamp for measurement
        uint32_t dist;     // 8 Odometer when direction changed
        uint32_t dist_abs; //12 Absolute distance travelled
    } __attribute__((packed));
    """
    rot = rotation(get16(frm), frm[2], get32(frm[4:]), get32(frm[8:]), get32(frm[12:]))
    return rot

def unpackErrorCount(frm):
    """
    struct errorcount
    {
        struct header hdr;
        uint32_t count;  // Counter value
        char name[24];   // ASCIIZ name
    } __attribute__((packed));
    """
    err = errorcount(get32(frm), getStr(frm[4:]))
    return err

# Defined addresses
ADDR_RPI = 0x01
ADDR_TEENSY = 0x02

# Protocol commands
# pylint: disable=C0326
Cmd = enum(\
    CMD_PING_QUERY    = 1,  #// timestamp exchange
    CMD_PONG_RESP     = 2,  #// timestamp exchange
    CMD_SET_SONAR_SEQ = 3,  #// up to 24 bytes. unused bytes are 0xff
    CMD_SONAR_STOP    = 4,  #// Stop Ultrasound sensors
    CMD_SONAR_START   = 5,  #// Start Ultrasound sensors
    CMD_SONAR_STATUS  = 6,  #// sensor id + distance
    CMD_WHEEL_STATUS  = 7,  #// direction, speed, odo
    CMD_WHEEL_RESET   = 8,  #// Clear odometer
    CMD_ERROR_COUNT   = 9,  #// Error counter name and value
    CMD_GET_COUNTERS  = 10, #// Ask teensy to send all non-zero counters
    CMD_SONAR_RETRY   = 11, #// Do a repeat nextSonar() (for internal stall recovery)
    CMD_SONAR_WAIT    = 12, #// Set waiting time between sonar pings in ms
          )
# pylint: enable=C0326

# Low-level packet helpers
def buildHeader(dst, src, cmd, buf=None):
    """Create header for Teensy message"""
    if buf is None:
        buf = []
    buf.append(dst)
    buf.append(src)
    buf.append(cmd)
    buf.append(0) # Reserved field
    return buf

def add32(buf, value):
    """Add a littleendian 32bit value to buffer"""
    buf.append(value & 0xff)
    value >>= 8
    buf.append(value & 0xff)
    value >>= 8
    buf.append(value & 0xff)
    value >>= 8
    buf.append(value & 0xff)
    return buf

def add16(buf, value):
    """Add a littleendian 16bit value to buffer"""
    buf.append(value & 0xff)
    value >>= 8
    buf.append(value & 0xff)
    return buf

class Sensorhub(object):
    """This class encapsulates the handling of the Teensy serial protocol.
    A user need only provide callback handlers for each type of incoming data
    and use the provided functions for sending commands to the Teensy."""
    # pylint: disable=too-many-instance-attributes
    # Packet framing bytes
    FRAME_START_STOP = 0x7e
    FRAME_DATA_ESCAPE = 0x7d
    FRAME_XOR = 0x20

    # pylint: disable=C0326
    # Possible receiver states
    rxs = enum(RS_BEGIN = 0,
               RS_DATA = 1, )

    # TX state machine states
    txs = enum(TS_BEGIN = 1,  # Nothing sent yet, deliver 0x7e
               TS_DATA = 2,   # Sending normal data
               TS_ESCAPE = 3, # Escape has been sent, txEscByte is next
              )
    # pylint: enable=C0326

    def __init__(self):
        self.rxEscapeFlag = False
        self.rxState = Sensorhub.rxs.RS_BEGIN
        self.rxRawData = []
        self.rxChecksum = 0
        self.txQueue = Queue.Queue() # The TX queue have byte arrays
        self.txCurrPacket = None
        self.txState = Sensorhub.txs.TS_BEGIN
        self.txEscByte = None
        self.ping = None
        self.pong = None
        self.wheel = None
        self.distance = None
        self.errorcount = None
        return

    def _rxDecodeFrame(self, frm):
        # pylint: disable=too-many-branches
        if len(frm) < 4:
            return
        hdr = header(frm[0], frm[1], frm[2], frm[3])
        frm = frm[4:]  # remove header and rxChecksum
        if hdr.cmd == Cmd.CMD_PING_QUERY:
            pp = unpackPingPong(frm)
            if self.ping != None:
                self.ping(pp)
        elif hdr.cmd == Cmd.CMD_PONG_RESP:
            pp = unpackPingPong(frm)
            if self.pong != None:
                self.pong(pp)
        elif hdr.cmd == Cmd.CMD_SET_SONAR_SEQ:
            pass
        elif hdr.cmd == Cmd.CMD_SONAR_STOP:
            pass
        elif hdr.cmd == Cmd.CMD_SONAR_START:
            pass
        elif hdr.cmd == Cmd.CMD_SONAR_STATUS:
            dist = unpackSonarStatus(frm)
            if self.distance != None:
                self.distance(dist)
        elif hdr.cmd == Cmd.CMD_WHEEL_STATUS:
            left = unpackRotation(frm)
            right = unpackRotation(frm[16:])
            if self.wheel != None:
                self.wheel(left, right)
        elif hdr.cmd == Cmd.CMD_WHEEL_RESET:
            pass
        elif hdr.cmd == Cmd.CMD_ERROR_COUNT:
            err = unpackErrorCount(frm)
            if self.errorcount != None:
                self.errorcount(err)
        return

    def _rxChecksumCalc(self, b):
        self.rxChecksum += b
        self.rxChecksum += (self.rxChecksum >> 8)
        self.rxChecksum = self.rxChecksum & 0xff
        return

    def _rxChecksumNew(self):
        self.rxChecksum = 0
        return

    def _rxHandleFrame(self):
        if self.rxRawData:
            if self.rxChecksum != 0xff:
                print("rxChecksum error:", self.rxChecksum, \
                "\nerror data:", len(self.rxRawData), self.rxRawData)
            else:
                self._rxDecodeFrame(self.rxRawData[:-1])
        return

    def _rxNewFrame(self):
        self.rxRawData = []
        self._rxChecksumNew()
        return

    def rxRawByte(self, character):
        """Handle incoming raw bytes and resolve byte stuffing and checksum handling"""
        b = ord(character)
        if self.rxState == Sensorhub.rxs.RS_BEGIN:
            if b == Sensorhub.FRAME_START_STOP:
                self.rxState = Sensorhub.rxs.RS_DATA
                self._rxNewFrame()
        elif self.rxState == Sensorhub.rxs.RS_DATA:
            if b == Sensorhub.FRAME_START_STOP:
                self._rxHandleFrame()
                self._rxNewFrame()
                return
            elif b == Sensorhub.FRAME_DATA_ESCAPE:
                self.rxEscapeFlag = True
                return
            else:
                if self.rxEscapeFlag:
                    self.rxEscapeFlag = False
                    b = b ^ Sensorhub.FRAME_XOR
                self._rxChecksumCalc(b)
                self.rxRawData.append(b)
        else:
            assert "Internal error"
        return

    # High-level packet sending helpers
    def sendPing(self):
        """Send ping command to teensy"""
        buf = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_PING_QUERY)
        buf = add32(buf, getMilliSeconds()) # timestamp1
        buf = add32(buf, 0)           # timestamp2
        self.txQueue.put(buf)
        return

    def sendSonarStop(self):
        """Stop Teensy sonar activity"""
        buf = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_SONAR_STOP)
        self.txQueue.put(buf)
        return

    def sendSonarStart(self):
        """Start Teensy sonar activity"""
        buf = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_SONAR_START)
        self.txQueue.put(buf)
        return

    def sendWheelReset(self):
        """Reset wheel odometer counters (left and right)"""
        buf = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_WHEEL_RESET)
        self.txQueue.put(buf)
        return

    def sendGetCounters(self):
        """Force transmission of all error counters from Teensy"""
        buf = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_GET_COUNTERS)
        self.txQueue.put(buf)
        return

    def sendSonarWait(self, pauseInMs):
        """Set minimum interval between sonar pulses"""
        buf = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_SONAR_WAIT)
        buf = add32(buf, pauseInMs)
        self.txQueue.put(buf)
        return

    def sendSonarSequence(self, sequence):
        """Send a new sonar sequence to Teensy"""
        buf = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_SET_SONAR_SEQ)
        l = len(sequence)
        if l > 0 and l <= 24:
            buf.append(l)
            buf = buf + sequence
            self.txQueue.put(buf)
        return

    def _txAppendChecksum(self, buf):
        """TX side of packet handling.
        Calculate checksum and add to buffer"""
        checksum = 0
        for b in buf:
            checksum += b
            checksum += checksum >> 8
            checksum = checksum & 0xff
        buf.append(0xff-checksum)
        return buf

    def _txGetNextPacket(self):
        """Set up new tx buffer if there no currently transmitting buffer"""
        if self.txCurrPacket is None or not self.txCurrPacket:
            if not self.txQueue.empty():
                self.txCurrPacket = self._txAppendChecksum(self.txQueue.get())
                return True
        return False

    def _txGetDataByte(self):
        """Get byte to send (if available) and return
        tuple (dataAvailable, byteToSend)"""
        if self.txCurrPacket is None:
            self._txGetNextPacket()
        if self.txCurrPacket != None:
            if self.txCurrPacket:
                b = self.txCurrPacket[0]
                self.txCurrPacket = self.txCurrPacket[1:]
                return [True, b]
        return [False, None]

    def txDataAvailable(self):
        """Indicate if there is data to transmit"""
        if self.txState != Sensorhub.txs.TS_BEGIN or \
          (self.txCurrPacket != None and self.txCurrPacket):
            return True
        return self._txGetNextPacket()


    def txGetByte(self):
        """Get next byte to transmit. Handles byte stuffing and checksum handling"""
        if self.txState == Sensorhub.txs.TS_BEGIN:
            if self.txDataAvailable():
                self.txState = Sensorhub.txs.TS_DATA
                return [True, Sensorhub.FRAME_START_STOP]
            return [False, None]

        elif self.txState == Sensorhub.txs.TS_DATA:
            dataAvailable, byte = self._txGetDataByte()
            if dataAvailable:
                if (byte == Sensorhub.FRAME_START_STOP) or \
                   (byte == Sensorhub.FRAME_DATA_ESCAPE):
                    self.txEscByte = byte ^ Sensorhub.FRAME_XOR
                    byte = Sensorhub.FRAME_DATA_ESCAPE
                    self.txState = Sensorhub.txs.TS_ESCAPE
                return [True, byte]
            else:
                # This packet done - next one back-to-back?
                if self._txGetNextPacket():
                    self.txState = Sensorhub.txs.TS_DATA
                else:
                    self.txState = Sensorhub.txs.TS_BEGIN
                return [True, Sensorhub.FRAME_START_STOP]

        elif self.txState == Sensorhub.txs.TS_ESCAPE:
            self.txState = Sensorhub.txs.TS_DATA
            return [True, self.txEscByte]

        return [False, None]

    # pylint: disable=too-many-arguments
    def setOutputHandlers(self, ping_handler, pong_handler, wheel_handler, \
                          distance_handler, errorcount_handler):
        """Install the callback handlers"""
        self.ping = ping_handler
        self.pong = pong_handler
        self.wheel = wheel_handler
        self.distance = distance_handler
        self.errorcount = errorcount_handler
        return
