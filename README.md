# Short Description

This is a simple port of MCP2221 DLL libraries into Python 3. It requires `ctypes` package to run. Unfortunately, DLL libraries provided directly by Microchip are supposed to work only under Windows environment (x86 or x64).

## Not cross-platform - solution?
If you need this to work under Linux and other OS (in Python), the most straightforward solution would be implementing packet communication using some HID library (eg. `hidapi`). The packets are described in the MCP2221 datasheet. I am just too lazy to do that.

## Where is UART?
You won't find any UART functions inside these DLL libraries, because COM port is automatically created after connection of MCP2221 to USB port, so you can use any freeware serial monitor program (PuTTY, Realterm etc.).
In Python, just use `serial` package.

## Monitoring USB
Also I recommend you to monitor attach/detach events of USB devices using eg. `pyudev` package and `MonitorObserver` class. The reason is the DLL handle doesn't close when you detach MCP2221 from USB port, so you could easily end up with blocked connection to MCP2221 until manual reset/reinsert into USB port.

## Code Example
```python
import mcp2221
import time

CUSTOM_VID = 0x0410
CUSTOM_PID = 0x2979

device = mcp2221.MCP2221()
print("DLL version:", device.getLibraryVersion()) # 2.2b

print("Number of connected MCP2221 chips:", device.getConnectedDevices(vid=CUSTOM_VID, pid=CUSTOM_PID))

# Now connect to the first of the devices
handle = device.openByIndex(0, vid=CUSTOM_VID, pid=CUSTOM_PID)

# Let's assume the GP0 pin is configured as GPIO output with LED connected
# Blink the LED quickly 20 times

for i in range(20):
    device.setGpioValues(handle, (0, device.NO_CHANGE, device.NO_CHANGE, device.NO_CHANGE));
    time.sleep(0.05)
    device.setGpioValues(handle, (1, device.NO_CHANGE, device.NO_CHANGE, device.NO_CHANGE));
    time.sleep(0.05)

# Close the connection in the end
device.close(handle)
```