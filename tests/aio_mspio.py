# aio_mspio.py
"""
Async-io based MSP (MultiWii Serial Protocol) I/O layer.
Hides every detail of the serial connection behind asyncio queues.

Public API (all coroutines unless explicitly stated otherwise):
----------------------------------------------------------------
async MSPIO.create(port: str, baudrate: int = 115200, *,
                   loop: asyncio.AbstractEventLoop | None = None,
                   read_queue_size: int = 0,
                   write_queue_size: int = 0,
                   min_interval: float = 1 / 100) -> "MSPIO"
    Factory. Opens the serial port and launches reader / writer tasks.

async MSPIO.send(code: int, payload: bytes | bytearray = b'') -> None
    Put a fully-formed MSP message on the writer queue.

async MSPIO.read() -> tuple[int, bytes]
    Await the next complete MSP message (code, payload).

def MSPIO.close() -> None
    Cancels tasks and closes the port immediately.

Queues
------
incoming : asyncio.Queue[(int, bytes)]   # (code, payload)
outgoing : asyncio.Queue[bytes]          # pre-built frames ready to write
"""
from __future__ import annotations

import asyncio
import sys
from typing import Final

try:
    import serial_asyncio  # pyserial-asyncio
except ModuleNotFoundError as e:  # pragma: no cover
    raise RuntimeError(
        "aio_mspio requires the 'pyserial-asyncio' package. Install with "
        '`pip install pyserial-asyncio`.'
    ) from e

# --------------------------------------------------------------------------- #
#                               MSP framing helpers                           #
# --------------------------------------------------------------------------- #

__all__ = ["MSPIO"]


_JUMBO_FRAME: Final[int] = 255

_CRC8_TABLE: Final[list[int]] = [
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
    0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
    0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
    0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
    0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
    0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
    0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
    0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
    0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
    0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
    0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9,
]


def _crc8(crc: int, ch: int) -> int:
    """Fast DVB-S2 CRC-8 used by MSP v2."""
    return _CRC8_TABLE[crc ^ ch]


def _prepare_msg(code: int, payload: bytes | bytearray = b"") -> bytes:
    """Return a fully-framed MSP v1/v2 packet."""
    if code <= 0xFF and len(payload) < _JUMBO_FRAME:
        mspv = 1
    else:
        mspv = 2

    if mspv == 1:
        length = len(payload)
        checksum = length ^ code
        for b in payload:
            checksum ^= b
        return (
            b"$M<"
            + bytes((length, code))
            + payload
            + bytes((checksum,))
        )

    # v2
    length = len(payload)
    lower_len = length & 0xFF
    upper_len = (length >> 8) & 0xFF
    lower_code = code & 0xFF
    upper_code = (code >> 8) & 0xFF
    header = bytearray(b"$X<\x00")  # "$", "X", "<", flag(0)
    header.extend((lower_code, upper_code, lower_len, upper_len))
    frame = header + payload
    crc = 0
    for b in frame[3:]:  # flag byte onward
        crc = _crc8(crc, b)
    frame.append(crc)
    return bytes(frame)


# --------------------------------------------------------------------------- #
#                           Async-io serial handler                           #
# --------------------------------------------------------------------------- #


class _MSPProtocol(asyncio.Protocol):
    """Internal protocol; feeds bytes to MSPIO without exposing serial."""

    def __init__(
        self,
        *,
        owner: "MSPIO",
        loop: asyncio.AbstractEventLoop,
    ) -> None:
        self._owner = owner
        self._loop = loop
        self._transport: asyncio.Transport | None = None
        self._buf = bytearray()
        self._state = 0
        self._mspv = 1
        self._expected = 0
        self._direction = 0
        self._code = 0
        self._checksum = 0
        self._flags = 0

    # asyncio.Protocol ----------------------------------------------------- #

    def connection_made(self, transport: asyncio.Transport) -> None:
        self._transport = transport

    def data_received(self, data: bytes) -> None:  # noqa: C901
        """Incremental state-machine MSP parser (v1 & v2)."""
        for byte in data:
            b = byte
            if self._state == 0:  # sync '$'
                if b == 36:
                    self._state = 1
            elif self._state == 1:  # 'M' or 'X'
                if b == 77:  # 'M'
                    self._mspv, self._state = 1, 2
                elif b == 88:  # 'X'
                    self._mspv, self._state = 2, 2
                else:
                    self._state = 0
            elif self._state == 2:  # direction
                if b == 33:  # '!'
                    self._state = 0  # unsupported, discard
                    continue
                self._direction = b
                if self._mspv == 1:
                    self._state = 3
                else:
                    self._state = 2.1
            elif self._state == 2.1:  # v2 flag (ignored)
                self._flags = b
                self._state = 2.2
            elif self._state == 2.2:  # v2 code low
                self._code = b
                self._state = 2.3
            elif self._state == 2.3:  # v2 code high
                self._code |= b << 8
                self._state = 3.1
            elif self._state == 3:  # v1 length
                self._expected = b
                self._checksum = b
                self._state = 4
            elif self._state == 3.1:  # v2 len low
                self._expected = b
                self._state = 3.2
            elif self._state == 3.2:  # v2 len high
                self._expected |= b << 8
                self._buf = bytearray()
                self._state = 7 if self._expected else 9
            elif self._state == 4:  # v1 code
                self._code = b
                self._checksum ^= b
                self._buf = bytearray()
                self._state = 7 if self._expected else 9
            elif self._state == 7:  # payload byte (common)
                self._buf.append(b)
                if self._mspv == 1:
                    self._checksum ^= b
                else:
                    # v2 CRC is handled later
                    pass
                if len(self._buf) == self._expected:
                    self._state = 9
            elif self._state == 9:  # checksum
                if self._mspv == 1:
                    if self._checksum == b:
                        self._dispatch()
                else:  # v2 CRC
                    crc = 0
                    crc = _crc8(crc, self._flags)
                    crc = _crc8(crc, self._code & 0xFF)
                    crc = _crc8(crc, (self._code >> 8) & 0xFF)
                    crc = _crc8(crc, self._expected & 0xFF)
                    crc = _crc8(crc, (self._expected >> 8) & 0xFF)
                    for p in self._buf:
                        crc = _crc8(crc, p)
                    if crc == b:
                        self._dispatch()
                self._state = 0  # always restart parser
            # Any unexpected byte resets state machine automatically by continuing loop

    def connection_lost(self, exc: BaseException | None) -> None:
        self._loop.call_soon_threadsafe(self._owner._reader_cancel)

    # helpers -------------------------------------------------------------- #

    def write(self, data: bytes) -> None:
        if self._transport is not None:
            self._transport.write(data)

    def _dispatch(self) -> None:
        """Push decoded packet into the public read queue."""
        self._loop.call_soon_threadsafe(
            self._owner._incoming.put_nowait,
            (self._code, bytes(self._buf)),
        )
        self._buf = bytearray()


# --------------------------------------------------------------------------- #
#                                    facade                                   #
# --------------------------------------------------------------------------- #


class MSPIO:
    """Owns the serial port. You touch queues only."""

    def __init__(
        self,
        port: str,
        baudrate: int,
        *,
        loop: asyncio.AbstractEventLoop,
        incoming: asyncio.Queue,
        outgoing: asyncio.Queue,
        min_interval: float,
        transport: asyncio.Transport,
        protocol: _MSPProtocol,
        tasks: tuple[asyncio.Task, asyncio.Task],
    ) -> None:
        self._port = port
        self._baud = baudrate
        self._loop = loop
        self._incoming = incoming
        self._outgoing = outgoing
        self._min_interval = min_interval
        self._transport = transport
        self._protocol = protocol
        self._reader_task, self._writer_task = tasks

    # --------------------------------------------------------------------- #
    #                            public  façade                             #
    # --------------------------------------------------------------------- #

    @property
    def incoming(self) -> asyncio.Queue:
        """Queue of (code, payload) tuples already checksum-verified."""
        return self._incoming

    async def read(self) -> tuple[int, bytes]:
        """Await next (code, payload)."""
        return await self._incoming.get()

    async def send(self, code: int, payload: bytes | bytearray = b"") -> None:
        """Queue an MSP frame for serial write."""
        frame = _prepare_msg(code, payload)
        await self._outgoing.put(frame)

    def close(self) -> None:
        """Abort background tasks and close immediately."""
        self._reader_cancel()
        self._writer_cancel()
        if self._transport:
            self._transport.close()

    # --------------------------------------------------------------------- #
    #                       factory & background tasks                      #
    # --------------------------------------------------------------------- #

    @classmethod
    async def create(
        cls,
        port: str,
        baudrate: int = 115200,
        *,
        loop: asyncio.AbstractEventLoop | None = None,
        read_queue_size: int = 0,
        write_queue_size: int = 0,
        min_interval: float = 1 / 100,
    ) -> "MSPIO":
        loop = loop or asyncio.get_running_loop()
        incoming: asyncio.Queue = asyncio.Queue(read_queue_size)
        outgoing: asyncio.Queue = asyncio.Queue(write_queue_size)
        protocol_factory = lambda: _MSPProtocol(owner=None, loop=loop)  # type: ignore[arg-type]

        transport, protocol = await serial_asyncio.create_serial_connection(
            loop, protocol_factory, url=port, baudrate=baudrate
        )
        # patch back-reference now that protocol exists
        protocol._owner = dummy = object()  # type: ignore[assignment]
        protocol._owner = None  # placeholder to satisfy mypy; overwritten below

        self = cls(
            port,
            baudrate,
            loop=loop,
            incoming=incoming,
            outgoing=outgoing,
            min_interval=min_interval,
            transport=transport,
            protocol=protocol,
            tasks=(None, None),  # type: ignore[arg-type]
        )
        protocol._owner = self

        reader_task = loop.create_task(self._reader())
        writer_task = loop.create_task(self._writer())

        self._reader_task = reader_task
        self._writer_task = writer_task
        return self

    # --------------------- internal tasks & helpers ---------------------- #

    async def _reader(self) -> None:
        """Runs until transport closes; protocol pushes packets."""
        try:
            await asyncio.Future()  # suspend forever; protocol drives queue
        except asyncio.CancelledError:
            pass

    async def _writer(self) -> None:
        last_write = 0.0
        try:
            while True:
                data = await self._outgoing.get()
                delta = self._loop.time() - last_write
                if delta < self._min_interval:
                    await asyncio.sleep(self._min_interval - delta)
                self._protocol.write(data)
                last_write = self._loop.time()
        except asyncio.CancelledError:
            pass

    # ------------------------- cancel helpers ---------------------------- #

    def _reader_cancel(self) -> None:
        if not self._reader_task.done():
            self._reader_task.cancel()

    def _writer_cancel(self) -> None:
        if not self._writer_task.done():
            self._writer_task.cancel()
