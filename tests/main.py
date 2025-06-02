import asyncio
from aio_mspio import MSPIO     # the file you saved earlier
from unavlib.enums import inav_enums
from unavlib.enums.msp_codes import MSPCodes
from unavlib.enums import msp_vars
from unavlib.modules.utils import inavutil

REQUEST_CODE = inavutil.msp.MSP_RAW_IMU              # whatever MSP code you want to send
REQUEST_PAYLOAD = b''           # or bytes/bytearray payload

async def main() -> None:
    # 1.  Open the port and launch background reader / writer tasks.
    msp = await MSPIO.create(
        port='/dev/ttyACM0',     # Windows: 'COM3', macOS: '/dev/tty.usbserial-xyz'
        baudrate=115200,
    )

    try:
        # 2.  Send a single MSP frame.
        await msp.send(REQUEST_CODE, REQUEST_PAYLOAD)

        # 3.  Await the reply (blocks until the next complete frame arrives).
        code, payload = await msp.read()
        print(f"RX code={code}  payload={payload.hex()}")

    finally:
        # 4.  Close cleanly -- this cancels the background tasks and
        #     closes the serial port.
        msp.close()

asyncio.run(main())
