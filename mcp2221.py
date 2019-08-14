# -*- coding: utf-8 -*-
"""
Microchip MCP2221 Python ctypes-based library (for Windows OS only)

Created on Aug 14 2019
@author: Vojtech Vrba (vrbavoj3@fel.cvut.cz)
"""

import ctypes
import sys

class MCP2221_Error(Exception):
    """
    Exception object for raising errors corresponding to return values of DLL functions.
    """
    def __init__(self, code):
        if code == 0:
            message = "E_NO_ERR (code " + str(code) + "): Operation was successful."
        elif code == -1:
            message = "E_ERR_UNKOWN_ERROR (code " + str(code) + "): Unknown error. This can happen in the getconnecteddevices, openbyindex or openbysn if searching through the connected hid devices fails."
        elif code == -2:
            message = "E_ERR_CMD_FAILED (code " + str(code) + "): The library indicates an unexpected device reply after being given a command: neither successful operation nor specific error code."
        elif code == -3:
            message = "E_ERR_INVALID_HANDLE (code " + str(code) + "): Invalid device handle usage attempt. The device is already closed or there is an issue with the device handles management in the application."
        elif code == -4:
            message = "E_ERR_INVALID_PARAMETER (code " + str(code) + "): At least one api parameter is not valid."
        elif code == -5:
            message = "E_ERR_INVALID_PASS (code " + str(code) + "): Invalid password string (length < 8)"
        elif code == -6:
            message = "E_ERR_PASSWORD_LIMIT_REACHED (code " + str(code) + "): An incorrect password was sent 3 times."
        elif code == -7:
            message = "E_ERR_FLASH_WRITE_PROTECTED (code " + str(code) + "): The command cannot be executed because the device is password protected or locked."
        elif code == -10:
            message = "E_ERR_NULL (code " + str(code) + "): Null pointer received."
        elif code == -11:
            message = "E_ERR_DESTINATION_TOO_SMALL (code " + str(code) + "): Destination string too small."
        elif code == -12:
            message = "E_ERR_INPUT_TOO_LARGE (code " + str(code) + "): The input string exceeds the maximum allowed size."
        elif code == -13:
            message = "E_ERR_FLASH_WRITE_FAILED (code " + str(code) + "): Flash write failed due to an unknown cause."
        elif code == -14:
            message = "E_ERR_MALLOC (code " + str(code) + "): Memory allocation error."
        elif code == -101:
            message = "E_ERR_NO_SUCH_INDEX (code " + str(code) + "): An attempt was made to open a connection to a non existing index (usually >= the number of connected devices."
        elif code == -103:
            message = "E_ERR_DEVICE_NOT_FOUND (code " + str(code) + "): No device with the provided vid/pid or SN has been found. This error can also occur during i2c/smbus operations if the device is disconnected from the usb before the operation is complete. The OpenBySn method will also return this code if a connection to a matching device is already open."
        elif code == -104:
            message = "E_ERR_INTERNAL_BUFFER_TOO_SMALL (code " + str(code) + "): One of the internal buffers of the function was too small."
        elif code == -105:
            message = "E_ERR_OPEN_DEVICE_ERROR (code " + str(code) + "): An error occurred when trying to get the device handle."
        elif code == -106:
            message = "E_ERR_CONNECTION_ALREADY_OPENED (code " + str(code) + "): Connection already opened."
        elif code == -107:
            message = "E_ERR_CLOSE_FAILED (code " + str(code) + "): File close operation failed due to unknown reasons."
        elif code == -301:
            message = "E_ERR_RAW_TX_TOO_LARGE (code " + str(code) + "): Low level communication error, shouldn't appear during normal operation."
        elif code == -302:
            message = "E_ERR_RAW_TX_COPYFAILED (code " + str(code) + "): Low level communication error, shouldn't appear during normal operation."
        elif code == -303:
            message = "E_ERR_RAW_RX_COPYFAILED (code " + str(code) + "): Low level communication error shouldn't appear during normal operation."
        elif code == -401:
            message = "E_ERR_INVALID_SPEED (code " + str(code) + "): I2C/smbus speed is not within accepted range of 46875 - 500000."
        elif code == -402:
            message = "E_ERR_SPEED_NOT_SET (code " + str(code) + "): The speed may fail to be set if an i2c/smbus operation is already in progress or in a timeout situation. The \"mcp2221_i2ccancelcurrenttransfer\" function can be used to free the bus before retrying to set the speed."
        elif code == -403:
            message = "E_ERR_INVALID_BYTE_NUMBER (code " + str(code) + "): The byte count is outside the accepted range for the attempted operation."
        elif code == -404:
            message = "E_ERR_INVALID_ADDRESS (code " + str(code) + "): Invalid slave address. If 7 bit addressing is used the maximum address value is 127."
        elif code == -405:
            message = "E_ERR_I2C_BUSY (code " + str(code) + "): The mcp2221 i2c/smbus engine is currently busy."
        elif code == -406:
            message = "E_ERR_I2C_READ_ERROR (code " + str(code) + "): Mcp2221 signaled an error during the i2c read operation."
        elif code == -407:
            message = "E_ERR_ADDRESS_NACK (code " + str(code) + "): Nack received for the slave address used."
        elif code == -408:
            message = "E_ERR_TIMEOUT (code " + str(code) + "): Either the \"timeout\" or \"retries\" value has been exceeded and no reply was received from the slave."
        elif code == -409:
            message = "E_ERR_TOO_MANY_RX_BYTES (code " + str(code) + "): The number of received data bytes is greater than requested."
        elif code == -410:
            message = "E_ERR_COPY_RX_DATA_FAILED (code " + str(code) + "): Could not copy the data received from the slave into the provided buffer."
        elif code == -411:
            message = "E_ERR_NO_EFFECT (code " + str(code) + "): The i2c engine (inside mcp2221) was already idle. The cancellation command had no effect."
        elif code == -412:
            message = "E_ERR_COPY_TX_DATA_FAILED (code " + str(code) + "): Failed to copy the data into the hid buffer."
        elif code == -413:
            message = "E_ERR_INVALID_PEC (code " + str(code) + "): The slave replied with a pec value different than the expected one."
        elif code == -414:
            message = "E_ERR_BLOCK_SIZE_MISMATCH (code " + str(code) + "): The slave sent a different value for the block size(byte count) than we expected"
        super().__init__(message)

class MCP2221(ctypes.WinDLL):
    """
    Main MCP2221 library object. Handles DLL loading and function calling.
    """
    ### Constants definitions
    # Microchip MCP2221 default Vendor ID value
    MCP2221_DEFAULT_VID = 0x4D8
    # Microchip MCP2221 default Product ID value
    MCP2221_DEFAULT_PID = 0xDD
    # read/write chip flash settings
    FLASH_SETTINGS = 0
    # read/write chip runtime settings
    RUNTIME_SETTINGS = 1
    # Do not change the existing value. For example you can alter a pin's function and mark the rest as "no_change" to maintain their existing configuration.
    NO_CHANGE = 0xFF
    # Pin configured as input/output
    MCP2221_GPFUNC_IO = 0
    # Pin configured as SSPND
    MCP2221_GP_SSPND = 1
    # pin configured as ClockOut
    MCP2221_GP_CLOCK_OUT = 1
    # pin configured for USBCFG
    MCP2221_GP_USBCFG = 1
    # pin configured for I2C LED
    MCP2221_GP_LED_I2C = 1
    # pin configured for UART RX LED
    MCP2221_GP_LED_UART_RX = 2
    # pin configured for ADC
    MCP2221_GP_ADC = 2
    # pin configured for UART TX LED
    MCP2221_GP_LED_UART_TX = 3
    # Pin configured for DAC function
    MCP2221_GP_DAC = 3
    # Pin configured for Interrupt On Change
    MCP2221_GP_IOC = 4
    # GPIO pin configured as input
    MCP2221_GPDIR_INPUT = 1
    # GPIO pin configured as output
    MCP2221_GPDIR_OUTPUT = 0
    
    def __init__(self):
        if sys.maxsize > 2**32:
            super().__init__('mcp2221_x64.dll')
        else:
            super().__init__('mcp2221_x86.dll')
            # For x86 DLL library, names of functions are malformed
            self.Mcp2221_ClearInterruptPinFlag = getattr(self, '_Mcp2221_ClearInterruptPinFlag@4')
            self.Mcp2221_Close = getattr(self, '_Mcp2221_Close@4')
            self.Mcp2221_CloseAll = getattr(self, '_Mcp2221_CloseAll@0')
            self.Mcp2221_GetAdcData = getattr(self, '_Mcp2221_GetAdcData@8')
            self.Mcp2221_GetAdcVref = getattr(self, '_Mcp2221_GetAdcVref@12')
            self.Mcp2221_GetClockSettings = getattr(self, '_Mcp2221_GetClockSettings@16')
            self.Mcp2221_GetConnectedDevices = getattr(self, '_Mcp2221_GetConnectedDevices@12')
            self.Mcp2221_GetDacValue = getattr(self, '_Mcp2221_GetDacValue@12')
            self.Mcp2221_GetDacVref = getattr(self, '_Mcp2221_GetDacVref@12')
            self.Mcp2221_GetFactorySerialNumber = getattr(self, '_Mcp2221_GetFactorySerialNumber@8')
            self.Mcp2221_GetGpioDirection = getattr(self, '_Mcp2221_GetGpioDirection@8')
            self.Mcp2221_GetGpioSettings = getattr(self, '_Mcp2221_GetGpioSettings@20')
            self.Mcp2221_GetGpioValues = getattr(self, '_Mcp2221_GetGpioValues@8')
            self.Mcp2221_GetHwFwRevisions = getattr(self, '_Mcp2221_GetHwFwRevisions@12')
            self.Mcp2221_GetInitialPinValues = getattr(self, '_Mcp2221_GetInitialPinValues@24')
            self.Mcp2221_GetInterruptEdgeSetting = getattr(self, '_Mcp2221_GetInterruptEdgeSetting@12')
            self.Mcp2221_GetInterruptPinFlag = getattr(self, '_Mcp2221_GetInterruptPinFlag@8')
            self.Mcp2221_GetLastError = getattr(self, '_Mcp2221_GetLastError@0')
            self.Mcp2221_GetLibraryVersion = getattr(self, '_Mcp2221_GetLibraryVersion@4')
            self.Mcp2221_GetManufacturerDescriptor = getattr(self, '_Mcp2221_GetManufacturerDescriptor@8')
            self.Mcp2221_GetProductDescriptor = getattr(self, '_Mcp2221_GetProductDescriptor@8')
            self.Mcp2221_GetSecuritySetting = getattr(self, '_Mcp2221_GetSecuritySetting@8')
            self.Mcp2221_GetSerialNumberDescriptor = getattr(self, '_Mcp2221_GetSerialNumberDescriptor@8')
            self.Mcp2221_GetSerialNumberEnumerationEnable = getattr(self, '_Mcp2221_GetSerialNumberEnumerationEnable@8')
            self.Mcp2221_GetUsbPowerAttributes = getattr(self, '_Mcp2221_GetUsbPowerAttributes@12')
            self.Mcp2221_GetVidPid = getattr(self, '_Mcp2221_GetVidPid@12')
            self.Mcp2221_I2cCancelCurrentTransfer = getattr(self, '_Mcp2221_I2cCancelCurrentTransfer@4')
            self.Mcp2221_I2cRead = getattr(self, '_Mcp2221_I2cRead@20')
            self.Mcp2221_I2cReadRestart = getattr(self, '_Mcp2221_I2cReadRestart@20')
            self.Mcp2221_I2cWrite = getattr(self, '_Mcp2221_I2cWrite@20')
            self.Mcp2221_I2cWriteNoStop = getattr(self, '_Mcp2221_I2cWriteNoStop@20')
            self.Mcp2221_I2cWriteRestart = getattr(self, '_Mcp2221_I2cWriteRestart@20')
            self.Mcp2221_OpenByIndex = getattr(self, '_Mcp2221_OpenByIndex@12')
            self.Mcp2221_OpenBySN = getattr(self, '_Mcp2221_OpenBySN@12')
            self.Mcp2221_Reset = getattr(self, '_Mcp2221_Reset@4')
            self.Mcp2221_SendPassword = getattr(self, '_Mcp2221_SendPassword@8')
            self.Mcp2221_SetAdcVref = getattr(self, '_Mcp2221_SetAdcVref@12')
            self.Mcp2221_SetAdvancedCommParams = getattr(self, '_Mcp2221_SetAdvancedCommParams@12')
            self.Mcp2221_SetClockSettings = getattr(self, '_Mcp2221_SetClockSettings@16')
            self.Mcp2221_SetDacValue = getattr(self, '_Mcp2221_SetDacValue@12')
            self.Mcp2221_SetDacVref = getattr(self, '_Mcp2221_SetDacVref@12')
            self.Mcp2221_SetGpioDirection = getattr(self, '_Mcp2221_SetGpioDirection@8')
            self.Mcp2221_SetGpioSettings = getattr(self, '_Mcp2221_SetGpioSettings@20')
            self.Mcp2221_SetGpioValues = getattr(self, '_Mcp2221_SetGpioValues@8')
            self.Mcp2221_SetInitialPinValues = getattr(self, '_Mcp2221_SetInitialPinValues@24')
            self.Mcp2221_SetInterruptEdgeSetting = getattr(self, '_Mcp2221_SetInterruptEdgeSetting@12')
            self.Mcp2221_SetManufacturerDescriptor = getattr(self, '_Mcp2221_SetManufacturerDescriptor@8')
            self.Mcp2221_SetPermanentLock = getattr(self, '_Mcp2221_SetPermanentLock@4')
            self.Mcp2221_SetProductDescriptor = getattr(self, '_Mcp2221_SetProductDescriptor@8')
            self.Mcp2221_SetSecuritySetting = getattr(self, '_Mcp2221_SetSecuritySetting@16')
            self.Mcp2221_SetSerialNumberDescriptor = getattr(self, '_Mcp2221_SetSerialNumberDescriptor@8')
            self.Mcp2221_SetSerialNumberEnumerationEnable = getattr(self, '_Mcp2221_SetSerialNumberEnumerationEnable@8')
            self.Mcp2221_SetSpeed = getattr(self, '_Mcp2221_SetSpeed@8')
            self.Mcp2221_SetUsbPowerAttributes = getattr(self, '_Mcp2221_SetUsbPowerAttributes@12')
            self.Mcp2221_SetVidPid = getattr(self, '_Mcp2221_SetVidPid@12')
            self.Mcp2221_SmbusBlockRead = getattr(self, '_Mcp2221_SmbusBlockRead@28')
            self.Mcp2221_SmbusBlockWrite = getattr(self, '_Mcp2221_SmbusBlockWrite@28')
            self.Mcp2221_SmbusBlockWriteBlockReadProcessCall = getattr(self, '_Mcp2221_SmbusBlockWriteBlockReadProcessCall@36')
            self.Mcp2221_SmbusReadByte = getattr(self, '_Mcp2221_SmbusReadByte@24')
            self.Mcp2221_SmbusReadWord = getattr(self, '_Mcp2221_SmbusReadWord@24')
            self.Mcp2221_SmbusReceiveByte = getattr(self, '_Mcp2221_SmbusReceiveByte@20')
            self.Mcp2221_SmbusSendByte = getattr(self, '_Mcp2221_SmbusSendByte@20')
            self.Mcp2221_SmbusWriteByte = getattr(self, '_Mcp2221_SmbusWriteByte@24')
            self.Mcp2221_SmbusWriteWord = getattr(self, '_Mcp2221_SmbusWriteWord@24')
        
    
    def clearInterruptPinFlag(self, handle):
        '''
        Clears the interrupt pin flag of a device.
        
        :param handle: the handle for the device for which the flag will be cleared.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ret = self.Mcp2221_ClearInterruptPinFlag(ctypes.c_void_p(handle))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def close(self, handle):
        '''
        Attempt to close a connection to a MCP2221.
        
        :param handle: The handle for the device we'll close the connection to.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ret = self.Mcp2221_Close(ctypes.c_void_p(handle))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def closeAll(self):
        '''
        Attempt to close all the currently opened MCP2221 connections. If successful, all existing handles will be set to INVALID_HANDLE_VALUE
        
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ret = self.Mcp2221_CloseAll()
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def getAdcData(self, handle):
        '''
        Reads the ADC data for all 3 analog pins.
        
        NOTE: the array must have a minimum length of 3.
        
        :param handle: the handle for the device.
        
        :returns adcDataArray: Tuple containing the 10-bit ADC values. Entry 0 will contain the value for ADC1, entry 1 - ADC2, entry 2 - ADC3.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        adcDataArray = (ctypes.c_uint * 3)()
        ret = self.Mcp2221_GetAdcData(ctypes.c_void_p(handle), ctypes.byref(adcDataArray))
        if ret == 0:
            return tuple(adcDataArray)
        raise MCP2221_Error(ret)
    
    
    def getAdcVref(self, handle, whichToGet):
        '''
        Gets the DAC voltage reference.
        
        :param handle: the handle for the device
        :param whichToGet: 0 to read Flash settings, >0 to read SRAM (runtime) settings
        
        :returns adcVref: The voltage reference for the ADC: 0 - Vdd, 1 - 1.024 V, 2 - 2.048 V, 3 - 4.096 V
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        adcVref = ctypes.c_ubyte()
        ret = self.Mcp2221_GetAdcVref(ctypes.c_void_p(handle), ctypes.c_ubyte(whichToGet), ctypes.byref(adcVref))
        if ret == 0:
            return adcVref.value
        raise MCP2221_Error(ret)
    
    
    def getClockSettings(self, handle, whichToGet):
        '''
        Gets the duty cycle and clock divider values for the clock out pin (if configured for this operation).
        
        :param handle: the handle for the device
        :param whichToGet: 0 to read Flash settings, >0 to read SRAM (runtime) settings
        
        :returns dutyCycle: value of the duty cycle of the waveform on the clock pin 0 - 0 %, 1 - 25 %, 2 - 50 %, 3 - 75 %
        :returns clockDivider: value of the clock divider. The value provided is a power of 2. The 48Mhz internal clock is divided by 2^value to obtain the output waveform frequency. The correspondence between the divider values and output frequencies are as follows: 1 - 24 MHz, 2 - 12 MHz, 3 - 6 MHz, 4 - 3 MHz, 5 - 1.5 MHz, 6 - 750 kHz, 7 - 375 kHz
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        dutyCycle = ctypes.c_ubyte()
        clockDivider = ctypes.c_ubyte()
        ret = self.Mcp2221_GetClockSettings(ctypes.c_void_p(handle), ctypes.c_ubyte(whichToGet), ctypes.byref(dutyCycle), ctypes.byref(clockDivider))
        if ret == 0:
            return dutyCycle.value, clockDivider.value
        raise MCP2221_Error(ret)
    
    
    def getConnectedDevices(self, vid=MCP2221_DEFAULT_VID, pid=MCP2221_DEFAULT_PID):
        '''
        Gets the number of connected MCP2221s with the provided VID & PID.
        
        :param vid: The vendor id of the MCP2221 devices to count
        :param pid: The product id of the MCP2221 devices to count
        
        :returns noOfDevs: The number of connected MCP2221s matching the provided VID and PID
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        noOfDevs = ctypes.c_uint()
        ret = self.Mcp2221_GetConnectedDevices(ctypes.c_uint(vid), ctypes.c_uint(pid), ctypes.byref(noOfDevs))
        if ret == 0:
            return noOfDevs.value
        raise MCP2221_Error(ret)
    
    
    def getDacValue(self, handle, whichToGet):
        '''
        Gets the DAC value.
        
        :param handle: the handle for the device
        :param whichToGet: 0 to read Flash settings, >0 to read SRAM (runtime) settings
        
        :returns dacValue: The DAC output value. Valid range is between 0 and 31.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        dacValue = ctypes.c_ubyte()
        ret = self.Mcp2221_GetDacValue(ctypes.c_void_p(handle), ctypes.c_ubyte(whichToGet), ctypes.byref(dacValue))
        if ret == 0:
            return dacValue.value
        raise MCP2221_Error(ret)
    
    
    def getDacVref(self, handle, whichToGet):
        '''
        Gets the DAC voltage reference.
        
        :param handle: the handle for the device
        :param whichToGet: 0 to read Flash settings, >0 to read SRAM (runtime) settings
        
        :returns dacVref: The voltage reference for the DAC: 0 - Vdd, 1 - 1.024 V, 2 - 2.048 V, 3 - 4.096 V
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        dacVref = ctypes.c_ubyte()
        ret = self.Mcp2221_GetDacVref(ctypes.c_void_p(handle), ctypes.c_ubyte(whichToGet), ctypes.byref(dacVref))
        if ret == 0:
            return dacVref.value
        raise MCP2221_Error(ret)
    
    
    def getFactorySerialNumber(self, handle):
        '''
        Returns the factory serial number of the device.
        
        Note: Use Marshal.GetLastWin32Error() to determine the error code if the function fails.
        
        :param handle: handle for the device
        
        :returns serialNumber: String containing the serial number
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        serialNumber = ctypes.create_unicode_buffer('\x00' * 30)
        ret = self.Mcp2221_GetFactorySerialNumber(ctypes.c_void_p(handle), serialNumber)
        if ret == 0:
            return serialNumber.value
        raise MCP2221_Error(ret)
    
    
    def getGpioDirection(self, handle):
        '''
        Gets the GPIO pin direction.
        
        NOTE: the output array must have a minimum length of 4.
        
        :param handle: the handle for the device
        
        :returns gpioDir: Array containing the direction of the IO pin 0 - output 1 - input 0xEF - GPx not set for GPIO operation
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        gpioDir = (ctypes.c_ubyte * 4)()
        ret = self.Mcp2221_GetGpioDirection(ctypes.c_void_p(handle), ctypes.byref(gpioDir))
        if ret == 0:
            return tuple(gpioDir)
        raise MCP2221_Error(ret)
    
    
    def getGpioSettings(self, handle, whichToGet):
        '''
        Gets the GPIO settings.
        
        NOTE: all output arrays must have a minimum length of 4.
        
        :param handle: the handle for the device
        :param whichToGet: 0 to read Flash settings, >0 to read SRAM (runtime) settings
        
        :returns pinFunctions: Array containing the values for the pin functions. pinFunction[i] will contain the value for pin GP"i. Possible values: 0 to 3. 0 - GPIO, 1 - Dedicated function, 2 - alternate function 0, 3 - alternate function 1, 4 - alternate function 2. 
        :returns pinDirections: Array containing the pin direction of the IO pins. 0 - output, 1 - input
        :returns outputValues: Array containing the value present on the output pins. 0 - logic low, 1 - logic high
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        pinFunctions = (ctypes.c_ubyte * 4)()
        pinDirections = (ctypes.c_ubyte * 4)()
        outputValues = (ctypes.c_ubyte * 4)()
        ret = self.Mcp2221_GetGpioSettings(ctypes.c_void_p(handle), ctypes.c_ubyte(whichToGet), ctypes.byref(pinFunctions), ctypes.byref(pinDirections), ctypes.byref(outputValues))
        if ret == 0:
            return tuple(pinFunctions), tuple(pinDirections), tuple(outputValues)
        raise MCP2221_Error(ret)
    
    
    def getGpioValues(self, handle):
        '''
        Gets the GPIO pin values.
        
        NOTE: the output array must have a minimum length of 4.
        
        :param handle: the handle for the device
        
        :returns gpioValues: Tuple containing the values present on the IO pins. 0 - logic low, 1 - logic high, 0xEE - GPx not set for GPIO operation
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        gpioValues = (ctypes.c_ubyte * 4)()
        ret = self.Mcp2221_GetGpioValues(ctypes.c_void_p(handle), ctypes.byref(gpioValues))
        if ret == 0:
            return tuple(gpioValues)
        raise MCP2221_Error(ret)
    
    
    def getHwFwRevisions(self, handle):
        '''
        Reads the hardware and firmware revision values from the device.
        
        NOTE: the output strings must have a minimum length of 2.
        
        :param handle: the handle for the device.
        
        :returns hardwareRevision: will contain the hardware revision string.
        
        :returns firmwareRevision: will contain the firmware revision string.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        hardwareRevision = ctypes.create_unicode_buffer('\x00' * 30)
        firmwareRevision = ctypes.create_unicode_buffer('\x00' * 30)
        ret = self.Mcp2221_GetHwFwRevisions(ctypes.c_void_p(handle), hardwareRevision, firmwareRevision)
        if ret == 0:
            return hardwareRevision.value, firmwareRevision.value
        raise MCP2221_Error(ret)
    
    
    def getInitialPinValues(self, handle):
        '''
        Gets the initial values for the special function pins: LEDUARTRX, LEDUARTTX, LEDI2C, SSPND and USBCFG
        
        :param handle: the handle for the device
        
        :returns ledUrxInitVal: this value represents the logic level signaled when no Uart Rx activity takes place (inactive level)
        
        :returns ledUtxInitVal: this value represents the logic level signaled when no Uart Tx activity takes place (inactive level)
        
        :returns ledI2cInitVal: this value represents the logic level signaled when no I2C traffic occurs (inactive level)
        
        :returns sspndInitVal: this value represents the logic level signaled when the device is not in suspend mode (inactive level)
        
        :returns usbCfgInitVal: this value represents the logic level signaled when the device is not usb configured (inactive level)
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ledUrxInitVal = ctypes.c_ubyte()
        ledUtxInitVal = ctypes.c_ubyte()
        ledI2cInitVal = ctypes.c_ubyte()
        sspndInitVal = ctypes.c_ubyte()
        usbCfgInitVal = ctypes.c_ubyte()
        ret = self.Mcp2221_GetInitialPinValues(ctypes.c_void_p(handle), ctypes.byref(ledUrxInitVal), ctypes.byref(ledUtxInitVal), ctypes.byref(ledI2cInitVal), ctypes.byref(sspndInitVal), ctypes.byref(usbCfgInitVal))
        if ret == 0:
            return ledUrxInitVal.value, ledUtxInitVal.value, ledI2cInitVal.value, sspndInitVal.value, usbCfgInitVal.value
        raise MCP2221_Error(ret)
    
    
    def getInterruptEdgeSetting(self, handle, whichToGet):
        '''
        Gets the interrupt pin trigger configuration.
        
        :param handle: the handle for the device
        :param whichToGet: 0 to read Flash settings, >0 to read SRAM (runtime) settings
        
        :returns interruptPinMode: value representing which edge will trigger the interrupt: 0 - none, 1 - positive edge, 2 - negative edge, 3 - both
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        interruptPinMode = ctypes.c_ubyte()
        ret = self.Mcp2221_GetInterruptEdgeSetting(ctypes.c_void_p(handle), ctypes.c_ubyte(whichToGet), ctypes.byref(interruptPinMode))
        if ret == 0:
            return interruptPinMode.value
        raise MCP2221_Error(ret)
    
    
    def getInterruptPinFlag(self, handle):
        '''
        Reads the interrupt pin flag value of a device.
        
        :param handle: the handle for the device.
        
        :returns flagValue: the value of the interrupt on change flag
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        flagValue = ctypes.c_ubyte()
        ret = self.Mcp2221_GetInterruptPinFlag(ctypes.c_void_p(handle), ctypes.byref(flagValue))
        if ret == 0:
            return flagValue.value
        raise MCP2221_Error(ret)
    
    
    def getLastError(self):
        """
        Description: Gets the last error value. Used only for the Open methods.
        """
        return int(self.Mcp2221_GetLastError())
    
    
    def getLibraryVersion(self):
        '''
        Returns the version number of the DLL
        
        
        :returns version: variable that will store the library version. The library version is a 10 character wchar string.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        version = ctypes.create_unicode_buffer('\x00' * 10)
        ret = self.Mcp2221_GetLibraryVersion(ctypes.byref(version))
        if ret == 0:
            return version.value
        raise MCP2221_Error(ret)
    
    
    def getManufacturerDescriptor(self, handle):
        '''
        Read USB Manufacturer Descriptor string from device.
        
        :param handle: The handle for the device.
        
        :returns manufacturerString: will contain the value of the USB Manufacturer Descriptor String. Note: the output string can contain up to 30 characters
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        manufacturerString = ctypes.create_unicode_buffer('\x00' * 30)
        ret = self.Mcp2221_GetManufacturerDescriptor(ctypes.c_void_p(handle), ctypes.byref(manufacturerString))
        if ret == 0:
            return manufacturerString.value
        raise MCP2221_Error(ret)
    
    
    def getProductDescriptor(self, handle):
        '''
        Read USB Product Descriptor string from device.
        
        :param handle: The handle for the device.
        
        :returns productString: will contain the value of the USB Product Descriptor String. Note: the output string can contain up to 30 characters
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        productString = ctypes.create_unicode_buffer('\x00' * 30)
        ret = self.Mcp2221_GetProductDescriptor(ctypes.c_void_p(handle), productString)
        if ret == 0:
            return productString.value
        raise MCP2221_Error(ret)
    
    
    def getSecuritySetting(self, handle):
        '''
        Gets the state of flash protection for the device
        
        :param handle: the handle for the device
        
        :returns securitySetting: the value of the chip security option: 0 - unsecured, 1 - password protected, 2 - permanently locked
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        securitySetting = ctypes.c_ubyte()
        ret = self.Mcp2221_GetSecuritySetting(ctypes.c_void_p(handle), ctypes.byref(securitySetting))
        if ret == 0:
            return securitySetting.value
        raise MCP2221_Error(ret)
    
    
    def getSerialNumberDescriptor(self, handle):
        '''
        Read USB Serial Number Descriptor string from device.
        
        :param handle: The handle for the device.
        
        :returns serialNumber: will contain the value of the USB Serial Number Descriptor String. Note: the output string can contain up to 30 characters
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        serialNumber = ctypes.create_unicode_buffer('\x00' * 30)
        ret = self.Mcp2221_GetSerialNumberDescriptor(ctypes.c_void_p(handle), ctypes.byref(serialNumber))
        if ret == 0:
            return serialNumber.value
        raise MCP2221_Error(ret)
    
    
    def getSerialNumberEnumerationEnable(self, handle):
        '''
        Gets the status of the Serial number enumeration bit.
        
        :param handle: the handle for the device
        
        :returns snEnumEnabled: determines if the serial number descriptor will be used during the USB enumeration of the CDC interface. If 1 - the serial number descriptor is used; if 0 - no serial number descriptor will be present during enumeration.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        snEnumEnabled = ctypes.c_ubyte()
        ret = self.Mcp2221_GetSerialNumberEnumerationEnable(ctypes.c_void_p(handle), ctypes.byref(snEnumEnabled))
        if ret == 0:
            return snEnumEnabled.value
        raise MCP2221_Error(ret)
    
    
    def getUsbPowerAttributes(self, handle):
        '''
        Gets the USB power attribute values.
        
        :param handle: the handle for the device
        
        :returns powerAttributes: the power attributes value from the USB descriptor. Bit meanings, based on the USB 2.0 spec: bit 7 - Reserved (Set to 1) (equivalent to Bus Powered), bit 6 - Self Powered, bit 5 - Remote Wakeup, bits 4..0 Reserved (reset to 0)
        
        :returns currentReq: the requested current value (mA); This value is expressed in multiples of 2mA.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        powerAttributes = ctypes.c_ubyte()
        currentReq = ctypes.c_int()
        ret = self.Mcp2221_GetUsbPowerAttributes(ctypes.c_void_p(handle), ctypes.byref(powerAttributes), ctypes.byref(currentReq))
        if ret == 0:
            return powerAttributes.value, currentReq.value
        raise MCP2221_Error(ret)
    
    
    def getVidPid(self, handle):
        '''
        Gets the VID and PID for the selected device.
        
        :param handle: the handle for the device
        
        :returns vid: The vendor id of the MCP2221 device
        
        :returns pid: The product id of the MCP2221 device
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        vid = ctypes.c_uint()
        pid = ctypes.c_uint()
        ret = self.Mcp2221_GetVidPid(ctypes.c_void_p(handle), ctypes.byref(vid), ctypes.byref(pid))
        if ret == 0:
            return vid.value, pid.value
        raise MCP2221_Error(ret)
    
    
    def i2cCancelCurrentTransfer(self, handle):
        '''
        Cancel the current I2C/SMBus transfer
        
        :param handle: The handle for the device.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ret = self.Mcp2221_I2cCancelCurrentTransfer(ctypes.c_void_p(handle))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def i2cRead(self, handle, bytesToRead, slaveAddress, use7bitAddress):
        '''
        Read I2C data from a slave.
        
        NOTE: if the "SetSpeed" function has not been called for the provided handle, the default speed of 100kbps will be configured and used. Otherwise, the speed will not be reconfigured.
        
        :param handle: The handle for the device.
        :param bytesToRead: the number of bytes to read from the slave. Valid range is between 1 and 65535.
        :param slaveAddress: 7bit or 8bit I2C slave address, depending on the value of the "use7bitAddress" flag. For 8 bit addresses, the R/W LSB of the address is set to 1 inside the function.
        :param use7bitAddress: if >0 - 7 bit address will be used for the slave. If 0 - 8 bit is used.
        
        :returns i2cRxData: buffer that will contain the data bytes read from the slave.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        readData = bytearray(bytesToRead)
        char_array = ctypes.c_ubyte * len(readData)
        ret = self.Mcp2221_I2cRead(ctypes.c_void_p(handle), ctypes.c_uint(bytesToRead), ctypes.c_ubyte(slaveAddress), ctypes.c_ubyte(use7bitAddress), char_array.from_buffer(readData))
        if ret == 0:
            return readData
        raise MCP2221_Error(ret)
    
    
    def i2cReadRestart(self, handle, bytesToRead, slaveAddress, use7bitAddress):
        '''
Read I2C data from a slave starting with a Repeated START.

        NOTE: 
            1. The speed must be set via the "SetSpeed" function before using this method. If the speed has not been set an error will be returned. 
            2. The SMBus Process Call command can be formed using I2cWriteNoStop followed by I2cReadRestart.
        
        :param handle: The handle for the device.
        :param bytesToRead: the number of bytes to read from the slave. Valid range is between 1 and 65535.
        :param slaveAddress: 7bit or 8bit I2C slave address, depending on the value of the "use7bitAddress" flag. For 8 bit addresses, the R/W LSB of the address is set to 1 inside the function.
        :param use7bitAddress: if >0 - 7 bit address will be used for the slave. If 0 - 8 bit is used. the slave.
        
        :returns i2cRxData: buffer that will contain the data bytes read from the slave.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        readData = bytearray(bytesToRead)
        char_array = ctypes.c_ubyte * len(readData)
        ret = self.Mcp2221_I2cReadRestart(ctypes.c_void_p(handle), ctypes.c_uint(bytesToRead), ctypes.c_ubyte(slaveAddress), ctypes.c_ubyte(use7bitAddress), char_array.from_buffer(readData))
        if ret == 0:
            return readData
        raise MCP2221_Error(ret)
    
    
    def i2cWrite(self, handle, bytesToWrite, slaveAddress, use7bitAddress, i2cTxData):
        '''
        Write I2C data to a slave without sending the STOP bit.
        
        NOTE: 
            1. The speed must be set via the "SetSpeed" function before using this method. If the speed has not been set an error will be returned. 
            2. The SMBus Process Call command can be formed using I2cWriteNoStop followed by I2cReadRestart.
        
        :param handle: The handle for the device.
        :param bytesToWrite: the number of bytes to write to the slave. Valid range is between 0 and 65535.
        :param slaveAddress: 7bit or 8bit I2C slave address, depending on the value of the "use7bitAddress" flag. For 8 bit addresses, the R/W LSB of the address is set to 0 inside the function.
        :param use7bitAddress: if >0 7 bit address will be used for the slave. If 0, 8 bit is used.
        :param i2cTxData: buffer that will contain the data bytes to be sent to the slave.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        char_array = ctypes.c_ubyte * len(i2cTxData)
        ret = self.Mcp2221_I2cWrite(ctypes.c_void_p(handle), ctypes.c_uint(bytesToWrite), ctypes.c_ubyte(slaveAddress), ctypes.c_ubyte(use7bitAddress), char_array.from_buffer(i2cTxData))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def i2cWriteNoStop(self, handle, bytesToWrite, slaveAddress, use7bitAddress, i2cTxData):
        '''
        Write I2C data to a slave without sending the STOP bit.
        
        NOTE: 
            1. The speed must be set via the "SetSpeed" function before using this method. If the speed has not been set an error will be returned. 
            2. The SMBus Process Call command can be formed using I2cWriteNoStop followed by I2cReadRestart.
        
        :param handle: The handle for the device.
        :param bytesToWrite: the number of bytes to write to the slave. Valid range is between 0 and 65535.
        :param slaveAddress: 7bit or 8bit I2C slave address, depending on the value of the "use7bitAddress" flag. For 8 bit addresses, the R/W LSB of the address is set to 0 inside the function.
        :param use7bitAddress: if >0 7 bit address will be used for the slave. If 0, 8 bit is used.
        :param i2cTxData: buffer that will contain the data bytes to be sent to the slave.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        char_array = ctypes.c_ubyte * len(i2cTxData)
        ret = self.Mcp2221_I2cWriteNoStop(ctypes.c_void_p(handle), ctypes.c_uint(bytesToWrite), ctypes.c_ubyte(slaveAddress), ctypes.c_ubyte(use7bitAddress), char_array.from_buffer(i2cTxData))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def i2cWriteRestart(self, handle, bytesToWrite, slaveAddress, use7bitAddress, i2cTxData):
        '''
        Write I2C data to a slave starting with a Repeated START.
        
        NOTE: The speed must be set via the "SetSpeed" function before using this method. If the speed has not been set an error will be returned.
        
        :param handle: The handle for the device.
        :param bytesToWrite: the number of bytes to write to the slave. Valid range is between 0 and 65535.
        :param slaveAddress: 7bit or 8bit I2C slave address, depending on the value of the "use7bitAddress" flag. For 8 bit addresses, the R/W LSB of the address is set to 0 inside the function.
        :param use7bitAddress: if > 0 7 bit address will be used for the slave. If 0 8 bit is used.
        :param i2cTxData: buffer that will contain the data bytes to be sent to the slave.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        char_array = ctypes.c_ubyte * len(i2cTxData)
        ret = self.Mcp2221_I2cWriteRestart(ctypes.c_void_p(handle), ctypes.c_uint(bytesToWrite), ctypes.c_ubyte(slaveAddress), ctypes.c_ubyte(use7bitAddress), char_array.from_buffer(i2cTxData))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def openByIndex(self, index, vid=MCP2221_DEFAULT_VID, pid=MCP2221_DEFAULT_PID):
        '''
        Attempt to open a connection with a MCP2221 with a specific index.
        
        NOTE: If the operation failed, call the GetLastError method to get the error code
        
        :param index: The index of the MCP2221 to connect to. This value ranges from 0 to n-1, where n is the number of connected devices. This value can be obtained from "GetConnectedDevices"
        :param VID: The vendor ID of the MCP2221 to connect to(Microchip default = 0x4D8)
        :param PID: The product ID of the MCP2221 to connect to(Microchip default = 0xDD)
        
        :returns handle: the handle value if the connection was successfully opened
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        handle = self.Mcp2221_OpenByIndex(ctypes.c_uint(vid), ctypes.c_uint(pid), ctypes.c_uint(index))
        if handle != -1:
            return handle
        raise MCP2221_Error(self.getLastError())
    
    
    def openBySN(self, serialNo, vid=MCP2221_DEFAULT_VID, pid=MCP2221_DEFAULT_PID):
        '''
        Attempt to open a connection with a MCP2221 with a specific serial number.
        
        NOTE: If the operation failed, call the GetLastError method to get the error code. If a connection to a matching device is already open, the function will fail with the "device not found" error.
        
        :param serialNo: The serial number of the MCP2221 we want to connect to. Maximum 30 character value.
        :param VID: The vendor ID of the MCP2221 to connect to.(Microchip default = 0x4D8)
        :param PID: The product ID of the MCP2221 to connect to. Microchip default = 0xDD)
        
        :returns handle: the handle value if the connection was successfully opened
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        buf = ctypes.create_unicode_buffer(serialNo)
        handle = self.Mcp2221_OpenBySN(ctypes.c_uint(vid), ctypes.c_uint(pid), ctypes.byref(buf))
        if handle != -1:
            return handle
        raise MCP2221_Error(self.getLastError())
    
    
    def reset(self, handle):
        '''
        Reset the MCP2221 and close its associated handle.
        
        :param handle: The handle for the device we'll reset. If successful, the handle will also be closed.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ret = self.Mcp2221_Reset(ctypes.c_void_p(handle))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def sendPassword(self, handle, password):
        '''
        Sends the access password to the device.
        
        NOTE: If 3 flash writes are attempted with an incorrect password, the chip won't accept any more passwords. This function doesn't validate the password, it just sends it to the device. The password is checked only during a flash write.
        
        :param handle: the handle for the device
        :param password: the password that will be sent to the device to unlock writing to flash. Must be an 8 character string.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        buffer = ctypes.create_string_buffer(bytes(password, encoding="utf-8"))
        ret = self.Mcp2221_SendPassword(ctypes.c_void_p(handle), ctypes.byref(buffer))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def setAdcVref(self, handle, whichToSet, adcVref):
        '''
        Sets the ADC voltage reference.
        
        :param handle: the handle for the device
        :param whichToSet: 0 to write Flash settings, >0 to write SRAM (runtime) settings
        :param adcVref: The voltage reference for the ADC: 0 - Vdd, 1 - 1.024 V, 2 - 2.048 V, 3 - 4.096 V
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ret = self.Mcp2221_SetAdcVref(ctypes.c_void_p(handle), ctypes.c_ubyte(whichToSet), ctypes.c_ubyte(adcVref))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def setAdvancedCommParams(self, handle, timeout, maxRetries):
        '''
        Set the time the MCP2221 will wait after sending the "read" command before trying to read back the data from the I2C/SMBus slave and the maximum number of retries if data couldn't be read back.
        
        :param handle: The handle for the device.
        :param timeout: amount of time (in ms) to wait for the slave to send back data. Default 3ms.
        :param maxRetries: the maximum amount of times we'll try to read data back from a slave. Default = 5
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ret = self.Mcp2221_SetAdvancedCommParams(ctypes.c_void_p(handle), ctypes.c_ubyte(timeout), ctypes.c_ubyte(maxRetries))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def setClockSettings(self, handle, dutyCycle, clockDivider):
        '''
        Sets the duty cycle and clock divider values for the clock out pin (if configured for this operation).
        
        :param handle: the handle for the device
        :param whichToSet: 0 to write Flash settings, >0 to write SRAM (runtime) settings
        :param dutyCycle: value of the duty cycle of the waveform on the clock pin: 0 - 0 %, 1 - 25 %, 2 - 50 %, 3 - 75 %
        :param clockDivider: value of the clock divider. The value provided is a power of 2. The 48Mhz internal clock is divided by 2^value to obtain the output waveform frequency. The correspondence between the divider values and output frequencies are as follows: 1 - 24 MHz, 2 - 12 MHz, 3 - 6 MHz, 4 - 3 MHz, 5 - 1.5 MHz, 6 - 750 kHz, 7 - 375 kHz
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ret = self.Mcp2221_SetClockSettings(ctypes.c_void_p(handle), ctypes.c_ubyte(dutyCycle), ctypes.c_ubyte(clockDivider))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def setDacValue(self, handle, whichToSet, dacValue):
        '''
        Sets the DAC value.
        
        :param handle: the handle for the device
        :param whichToSet: 0 to write Flash settings, >0 to write SRAM (runtime) settings
        :param dacValue: The DAC output value. Valid range is between 0 and 31.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ret = self.Mcp2221_SetDacValue(ctypes.c_void_p(handle), ctypes.c_ubyte(whichToSet), ctypes.c_ubyte(dacValue))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def setDacVref(self, handle, whichToSet, dacVref):
        '''
        Sets the DAC voltage reference.
        
        :param handle: the handle for the device
        :param whichToSet: 0 to write Flash settings, >0 to write SRAM (runtime) settings
        :param dacVref: The voltage reference for the DAC: 0 - Vdd, 1 - 1.024 V, 2 - 2.048 V, 3 - 4.096 V
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ret = self.Mcp2221_SetDacVref(ctypes.c_void_p(handle), ctypes.c_ubyte(whichToSet), ctypes.c_ubyte(dacVref))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def setGpioDirection(self, handle, gpioDir):
        '''
        Sets the GPIO pin direction.
        
        :param handle: the handle for the device
        :param gpioDir: Tuple containing the direction of each IO pin: 0 - output, 1 - input
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        buffer = ctypes.c_ubyte * 4
        ret = self.Mcp2221_SetGpioDirection(ctypes.c_void_p(handle), buffer.from_buffer(bytearray(gpioDir)))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def setGpioSettings(self, handle, whichToSet, pinFunctions, pinDirections, outputValues):
        '''
        Sets the GPIO settings.
        
        :param handle: the handle for the device
        :param whichToSet: 0 to write Flash settings, >0 to read SRAM (runtime) settings
        :param pinFunctions: Tuple containing the values for the pin functions. pinFunction[i] will contain the value for pin GP"i". Possible values: 0 to 3. 0 - GPIO, 1 - Dedicated function, 2 - alternate function 0, 3 - alternate function 1, 4 - alternate function 2, 0xFF - leave the pin unchanged.
        :param pinDirections: Tuple containing the pin direction of the IO pins. 0 - output 1 - input 0xff - leave unchanged
        :param outputValues: Tuple containing the value present on the output pins. 0 - logic low 1 - logic high 0xff - leave unchanged
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        buffer = ctypes.c_ubyte * 4
        ret = self.Mcp2221_SetGpioSettings(ctypes.c_void_p(handle), ctypes.c_ubyte(whichToSet), buffer.from_buffer(bytearray(pinFunctions)), buffer.from_buffer(bytearray(pinDirections)), buffer.from_buffer(bytearray(outputValues)))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def setGpioValues(self, handle, gpioValues):
        '''
        Sets the runtime GPIO pin values. Sets the runtime GPIO pin directions; flash values are not changed.
        
        :param handle: the handle for the device
        :param gpioValues: Array containing the value of the output IO pins. 0 - logic low, 1 - logic high, 0xFF - no change
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        buffer = ctypes.c_ubyte * 4
        ret = self.Mcp2221_SetGpioValues(ctypes.c_void_p(handle), buffer.from_buffer(bytearray(gpioValues)))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def setInitialPinValues(self, handle, ledUrxInitVal, ledUtxInitVal, ledI2cInitVal, sspndInitVal, usbCfgInitVal):
        '''
        Sets the initial values for the special function pins: LEDUARTRX, LEDUARTTX, LEDI2C, SSPND and USBCFG. The settings are saved to flash and take effect after a device reset.
        
        NOTE: Accepted values for the logic levels are 0(low) and 1(high), 0xff (leave unchanged)
        
        :param handle: the handle for the device
        :param ledUrxInitVal: this value represents the logic level signaled when no Uart Rx activity takes place (inactive level)
        :param ledUtxInitVal: this value represents the logic level signaled when no Uart Tx activity takes place (inactive level)
        :param ledI2cInitVal: this value represents the logic level signaled when no I2C traffic occurs (inactive level)
        :param sspndInitVal: this value represents the logic level signaled when the device is not in suspend mode (inactive level)
        :param usbCfgInitVal: this value represents the logic level signaled when the device is not usb configured (inactive level)
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ret = self.Mcp2221_SetInitialPinValues(ctypes.c_void_p(handle), ctypes.c_uint(ledUrxInitVal), ctypes.c_uint(ledUtxInitVal), ctypes.c_uint(ledI2cInitVal), ctypes.c_uint(sspndInitVal), ctypes.c_uint(usbCfgInitVal))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def setInterruptEdgeSetting(self, handle, whichToSet, interruptPinMode):
        '''
        Sets the interrupt pin trigger configuration.
        
        :param handle: the handle for the device
        :param whichToSet: 0 to write Flash settings, >0 to write SRAM (runtime) settings
        :param interruptPinMode: value representing which edge will trigger the interrupt: 0 - none, 1 - positive edge, 2 - negative edge, 3 - both
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ret = self.Mcp2221_SetInterruptEdgeSetting(ctypes.c_void_p(handle), ctypes.c_ubyte(whichToSet), ctypes.c_ubyte(interruptPinMode))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def setManufacturerDescriptor(self, handle, manufacturerString):
        '''
        Write USB Manufacturer Descriptor string to the device.
        
        :param handle: The handle for the device.
        :param manufacturerString: will contain the value of the USB Manufacturer Descriptor String. Note: the input string can contain a maximum of 30 characters
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        buffer = ctypes.create_unicode_buffer(manufacturerString)
        ret = self.Mcp2221_SetManufacturerDescriptor(ctypes.c_void_p(handle), ctypes.byref(buffer))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def setPermanentLock(self, handle):
        '''
        Permanently lock the device flash settings -- this action CAN'T be undone.
        
        !!! WARNING !!! -- USE THIS FUNCTION WITH GREAT CAUTION. THE CHIP FLASH SETTINGS (boot-up defaults) CANNOT BE CONFIGURED AFTER THIS FUNCTION HAS BEEN INVOKED!!
        
        :param handle: the handle for the device to be locked
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ret = self.Mcp2221_SetPermanentLock(ctypes.c_void_p(handle))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def setProductDescriptor(self, handle, productString):
        '''
        Write USB Product Descriptor string to the device.
        
        :param handle: The handle for the device.
        :param productString: will contain the value of the USB Product Descriptor String. Note: the input string can contain a maximum of 30 characters
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        buffer = ctypes.create_unicode_buffer(productString)
        ret = self.Mcp2221_SetProductDescriptor(ctypes.c_void_p(handle), ctypes.byref(buffer))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def setSecuritySetting(self, handle, securitySetting, currentPassword, newPassword):
        '''
        Sets the state of flash protection for the device
        
        :param handle: the handle for the device
        :param securitySetting: the value of the chip security option. If any other values are used, the E_ERR_INVALID_PARAMETER (-4) error is returned. 0 - disable password protection, 1 - enable password protection, 0xff - change current password
        :param currentPassword: the value for the currently set password. This is used for when the password "disable" or "change" operations are taking place.
        :param newPassword: the value for the new password. Must be an 8 character string. This is only for the "enable" or "change" operations.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        buffer1 = ctypes.create_string_buffer(bytes(currentPassword, encoding="utf-8"))
        buffer2 = ctypes.create_string_buffer(bytes(newPassword, encoding="utf-8"))
        ret = self.Mcp2221_SetSecuritySetting(ctypes.c_void_p(handle), ctypes.c_ubyte(securitySetting), ctypes.byref(buffer1), ctypes.byref(buffer2))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def setSerialNumberDescriptor(self, handle, serialNumber):
        '''
        Write USB Serial Number Descriptor string to the device.
        
        :param handle: The handle for the device.
        :param serialNumber: will contain the value of the USB Serial Number Descriptor String. Note: the input string can contain a maximum of 30 characters
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        buffer = ctypes.create_unicode_buffer(serialNumber)
        ret = self.Mcp2221_SetSerialNumberDescriptor(ctypes.c_void_p(handle), ctypes.byref(buffer))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def setSerialNumberEnumerationEnable(self, handle, snEnumEnabled):
        '''
        Sets the status of the Serial number enumeration bit.
        
        :param handle: the handle for the device
        :param snEnumEnabled: determines if the serial number descriptor will be used during the USB enumeration of the CDC interface. If 1 - the serial number descriptor is used; if 0 - no serial number descriptor will be present during enumeration.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ret = self.Mcp2221_SetSerialNumberEnumerationEnable(ctypes.c_void_p(handle), ctypes.c_ubyte(snEnumEnabled))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def setSpeed(self, handle, speed):
        '''
        Set the communication speed for I2C/SMBus operations.
        
        NOTE: The speed may fail to be set if an I2C/SMBus operation is already in progress or in a timeout situation. The "I2cCancelCurrentTransfer" function can be used to free the bus before retrying to set the speed.
        
        :param handle: The handle for the device.
        :param speed: the communication speed. Accepted values are between 46875 and 500000.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ret = self.Mcp2221_SetSpeed(ctypes.c_void_p(handle), ctypes.c_uint(speed))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def setUsbPowerAttributes(self, handle, powerAttributes, currentReq):
        '''
        Sets the USB power attribute values.
        
        NOTE: For the PowerAttributes parameter, bits 7 and 0-4 are automatically set to the correct reserved" value.
        
        :param handle: the handle for the device
        :param powerAttributes: the power attributes value from the USB descriptor. Bit meanings, based on the USB 2.0 spec: bit 7 - Reserved (Set to 1) (equivalent to Bus Powered), bit 6 - Self Powered, bit 5 - Remote Wakeup, bits 4..0 Reserved (reset to 0). The following constants can be OR'd to set this value: MCP2221_USB_SELF, MCP2221_USB_REMOTE, MCP2221_USB_BUS
        :param currentReq: the requested current value (mA); This value is expressed in multiples of 2mA. Valid range is between 0 and 500mA. If an odd value is used, it will be rounded down to the closest even value (ex currentReq = 201mA will result in a 200mA current request).
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ret = self.Mcp2221_SetUsbPowerAttributes(ctypes.c_void_p(handle), ctypes.c_ubyte(powerAttributes), ctypes.c_int(currentReq))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    def setVidPid(self, handle, vid, pid):
        '''
        Sets the VID and PID for the selected device.
        
        NOTE: the new VID/PID values will take effect after a device reset.
        
        :param handle: the handle for the device
        :param vid: The vendor id to be set
        :param pid: The product id to be set
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ret = self.Mcp2221_SetVidPid(ctypes.c_void_p(handle), ctypes.c_uint(vid), ctypes.c_uint(pid))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def smbusBlockRead(self, handle, slaveAddress, use7bitAddress, usePec, command, byteCount):
        '''
        SMBus Block Read.
        
        NOTE: If the "SetSpeed" function has not been called for the provided handle, the default speed of 100kbps will be configured and used. Otherwise, the speed will not be reconfigured.
        
        :param handle: The handle for the device.
        :param slaveAddress: 7bit or 8bit SMBus slave address, depending on the value of the "use7bitAddress" flag. For 8 bit addresses, the R/W LSB of the address is set to 1 inside the function.
        :param use7bitAddress: if >0, 7 bit address will be used for the slave. If 0, 8 bit is used.
        :param usePec: if >0, Packet Error Checking (PEC) will be used. The CRC8 values is computed for the SMBus packet compared with the PEC byte sent by the slave. If the two values differ the function returns an error code.
        :param command: The command code byte.
        :param byteCount: (block size) the number of data bytes that the slave will send to the master. Valid range is between 1 and 255 bytes. If there is a mismatch between this value and the byteCount the slave reports that it will send, an error will be returned.
        
        :returns readData: Array containing the data bytes read from the slave. If PEC is used, the last data byte will be the PEC byte received from the slave so the array should have a length of n+1, where n is the block size.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        readData = bytearray(byteCount)
        char_array = ctypes.c_ubyte * len(readData)
        ret = self.Mcp2221_SmbusBlockRead(ctypes.c_void_p(handle), ctypes.c_ubyte(slaveAddress), ctypes.c_ubyte(use7bitAddress), ctypes.c_ubyte(usePec), ctypes.c_ubyte(command), ctypes.c_ubyte(byteCount), char_array.from_buffer(readData))
        if ret == 0:
            return readData
        raise MCP2221_Error(ret)
    
    
    def smbusBlockWrite(self, handle, slaveAddress, use7bitAddress, usePec, command, byteCount, data):
        '''
        SMBus Block Write. The first byte of a Block Write operation is the command code, followed by the number of data bytes, then data bytes.
        
        NOTE: If the "SetSpeed" function has not been called for the provided handle, the default speed of 100kbps will be configured and used. Otherwise, the speed will not be reconfigured.
        
        :param handle: The handle for the device.
        :param slaveAddress: 7bit or 8bit SMBus slave address, depending on the value of the "use7bitAddress" flag. For 8 bit addresses, the R/W LSB of the address is set to 0 inside the function.
        :param use7bitAddress: if > 0 - 7 bit address will be used for the slave. If 0 - 8 bit is used.
        :param usePec: if > 0 - Packet Error Checking (PEC) will be used. A PEC byte containing the CRC8 for the sent message is appended after the data byte.
        :param command: The command code byte.
        :param byteCount: the number of data bytes that will be sent to the slave. Valid range is between 0 and 255 bytes, conforming to the smbus v3 specification.
        :param data: Array containing the data bytes to be sent to the slave.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        char_array = ctypes.c_ubyte * byteCount
        ret = self.Mcp2221_SmbusBlockWrite(ctypes.c_void_p(handle), ctypes.c_ubyte(slaveAddress), ctypes.c_ubyte(use7bitAddress), ctypes.c_ubyte(usePec), ctypes.c_ubyte(command), ctypes.c_ubyte(byteCount), char_array.from_buffer(data))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def smbusBlockWriteBlockReadProcessCall(self, handle, slaveAddress, use7bitAddress, usePec, command, writeByteCount, writeData, readByteCount):
        '''
        SMBus Block Write Block Read Process Call.
        
        NOTE: If the "SetSpeed" function has not been called for the provided handle, the default speed of 100kbps will be configured and used.Otherwise, the speed will not be reconfigured.speed of 100kbps will be configured and used.
        
        :param handle: The handle for the device.
        :param slaveAddress: 7bit or 8bit SMBus slave address, depending on the value of the "use7bitAddress" flag. For 8 bit addresses, the R/W LSB of the address is set to 0 inside the function.
        :param use7bitAddress: if >0, 7 bit address will be used for the slave. If 0, 8 bit is used.
        :param usePec: if >0, Packet Error Checking (PEC) will be used. The CRC8 values is computed for the SMBus packet and compared with the PEC byte sent by the slave. If the two values differ the function returns an error code.
        :param command: The command code byte.
        :param writeByteCount: the number of data bytes that will be sent to the slave. The total data payload must not exceed 255 bytes (writeByteCount + readByteCound <= 255) and writeByteCount > 0
        :param writeData: array containing the data bytes to be sent to the slave.
        :param readByteCount: the number of data bytes that the slave will send to the master. If there is a mismatch between this value and the readByteCount the slave reports that it will send, an error will be returned. The total data payload must not exceed 255 bytes writeByteCount + readByteCound <= 255) and readByteCount > 0
        
        :returns readData: Array containing the data bytes read from the slave. If PEC is used, the last data byte will be the PEC byte received from the slave so the array should have a length of n+1, where n is the readByteCount size.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        readData = bytearray(readByteCount)
        char_array_read = ctypes.c_ubyte * len(readData)
        char_array_write = ctypes.c_ubyte * writeByteCount
        ret = self.Mcp2221_SmbusBlockWriteBlockReadProcessCall(ctypes.c_void_p(handle), ctypes.c_ubyte(slaveAddress), ctypes.c_ubyte(use7bitAddress), ctypes.c_ubyte(usePec), ctypes.c_ubyte(command), ctypes.c_ubyte(writeByteCount), char_array_write.from_buffer(writeData), ctypes.c_ubyte(readByteCount), char_array_read.from_buffer(readData))
        if ret == 0:
            return readData
        raise MCP2221_Error(ret)
    
    
    def smbusReadByte(self, handle, slaveAddress, use7bitAddress, usePec, command):
        '''
        SMBus Read Byte. First Write the command byte to the slave, then read one data byte back.
        
        NOTE: If the "SetSpeed" function has not been called for the provided handle, the default speed of 100kbps will be configured and used.Otherwise, the speed will not be reconfigured.
        
        :param handle: The handle for the device.
        :param slaveAddress: 7bit or 8bit SMBus slave address, depending on the value of the "use7bitAddress" flag. For 8 bit addresses, the R/W LSB of the address is set to 1 inside the function.
        :param use7bitAddress: if >0, 7 bit address will be used for the slave. If 0, 8 bit is used.
        :param usePec: if >0, Packet Error Checking (PEC) will be used.
        :param command: The command code byte.
        
        :returns readByte: The data byte received from the slave
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        readByte = ctypes.c_ubyte()
        ret = self.Mcp2221_SmbusReadByte(ctypes.c_void_p(handle), ctypes.c_ubyte(slaveAddress), ctypes.c_ubyte(use7bitAddress), ctypes.c_ubyte(usePec), ctypes.c_ubyte(command), ctypes.byref(readByte))
        if ret == 0:
            return readByte.value
        raise MCP2221_Error(ret)
    
    
    def smbusReadWord(self, handle, slaveAddress, use7bitAddress, usePec, command):
        '''
        SMBus Read Word. First Write the command byte to the slave, then read one data byte back.
        
        NOTE: If the "SetSpeed" function has not been called for the provided handle, the default speed of 100kbps will be configured and used. Otherwise, the speed will not be reconfigured.
        
        :param handle: The handle for the device.
        :param slaveAddress: 7bit or 8bit SMBus slave address, depending on the value of the "use7bitAddress" flag.For 8 bit addresses, the R/W LSB of the address is set to 1 inside the function.
        :param use7bitAddress: if >0, 7 bit address will be used for the slave. If 0, 8 bit is used.
        :param usePec: if >0 Packet Error Checking (PEC) will be used.
        :param command: The command code byte.
        
        :returns readData: Buffer that will store the read data word. readData[0] - data_byte_low readData[1] - data_byte_high
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        readData = bytearray(2)
        char_array = ctypes.c_ubyte * len(readData)
        ret = self.Mcp2221_SmbusReadWord(ctypes.c_void_p(handle), ctypes.c_ubyte(slaveAddress), ctypes.c_ubyte(use7bitAddress), ctypes.c_ubyte(usePec), ctypes.c_ubyte(command), char_array.from_buffer(readData))
        if ret == 0:
            return readData
        raise MCP2221_Error(ret)
    
    
    def smbusReceiveByte(self, handle, slaveAddress, use7bitAddress, usePec):
        '''
        SMBus Receive Byte. Read one data byte back.

        NOTE: If the "SetSpeed" function has not been called for the provided handle, the default speed of 100kbps will be configured and used.Otherwise, the speed will not be reconfigured.
        
        :param handle: The handle for the device.
        :param slaveAddress: 7bit or 8bit SMBus slave address, depending on the value of the "use7bitAddress" flag. For 8 bit addresses, the R/W LSB of the address is set to 1 inside the function.
        :param use7bitAddress: if >0, 7 bit address will be used for the slave. If 0, 8 bit is used.
        :param usePec: if >0, Packet Error Checking (PEC) will be used.
        
        :returns readByte: The data byte received from the slave
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        readByte = ctypes.c_ubyte()
        ret = self.Mcp2221_SmbusReceiveByte(ctypes.c_void_p(handle), ctypes.c_ubyte(slaveAddress), ctypes.c_ubyte(use7bitAddress), ctypes.c_ubyte(usePec), ctypes.byref(readByte))
        if ret == 0:
            return readByte.value
        raise MCP2221_Error(ret)
    
    
    def smbusSendByte(self, handle, slaveAddress, use7bitAddress, usePec, data):
        '''
        SMBus Send byte. Sends one data byte.
        
        NOTE: If the "SetSpeed" function has not been called for the provided handle, the default speed of 100kbps will be configured and used. Otherwise, the speed will not be reconfigured.
        
        :param handle: The handle for the device.
        :param slaveAddress: 7bit or 8bit SMBus slave address, depending on the value of the "use7bitAddress" flag.For 8 bit addresses, the R/W LSB of the address is set to 0 inside the function.
        :param use7bitAddress: if >0, 7 bit address will be used for the slave. If 0, 8 bit is used.
        :param usePec: if >0 Packet Error Checking (PEC) will be used. A PEC byte containing the CRC8 value for the sent message is appended after the data byte.
        :param data: The data byte.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ret = self.Mcp2221_SmbusSendByte(ctypes.c_void_p(handle), ctypes.c_ubyte(slaveAddress), ctypes.c_ubyte(use7bitAddress), ctypes.c_ubyte(usePec), ctypes.c_ubyte(data))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def smbusWriteByte(self, handle, slaveAddress, use7bitAddress, usePec, command, data):
        '''
        SMBus write byte. The first byte of a Write Byte operation is the command code. The next one is the data to be written.
        
        NOTE: If the "SetSpeed" function has not been called for the provided handle, the default speed of 100kbps will be configured and used. Otherwise, the speed will not be reconfigured.
        
        :param handle: The handle for the device.
        :param slaveAddress: 7bit or 8bit SMBus slave address, depending on the value of the "use7bitAddress" flag.For 8 bit addresses, the R/W LSB of the address is set to 0 inside the function.
        :param use7bitAddress: if >0, 7 bit address will be used for the slave. If 0, 8 bit is used.
        :param usePec: if >0 Packet Error Checking (PEC) will be used. A PEC byte containing the CRC8 value for the sent message is appended after the data byte.
        :param command: The command code byte.
        :param data: The data byte.
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        ret = self.Mcp2221_SmbusWriteByte(ctypes.c_void_p(handle), ctypes.c_ubyte(slaveAddress), ctypes.c_ubyte(use7bitAddress), ctypes.c_ubyte(usePec), ctypes.c_ubyte(command), ctypes.c_ubyte(data))
        if ret != 0:
            raise MCP2221_Error(ret)
    
    
    def smbusWriteWord(self, handle, slaveAddress, use7bitAddress, usePec, command, data):
        '''
        SMBus write word. The first byte of a Write Byte operation is the command code, followed by the data_byte_low then data_byte_high.
        
        NOTE: If the "SetSpeed" function has not been called for the provided handle, the default speed of 100kbps will be configured and used. Otherwise, the speed will not be reconfigured.
        
        :param handle: The handle for the device.
        :param slaveAddress: 7bit or 8bit SMBus slave address, depending on the value of the "use7bitAddress" flag. For 8 bit addresses, the R/W LSB of the address is set to 0 inside the function.
        :param use7bitAddress: if >0, 7 bit address will be used for the slave. If 0, 8 bit is used.
        :param usePec: if >0, Packet Error Checking (PEC) will be used. A PEC byte containing the CRC8 value for the sent message is appended after the data byte.
        :param command: The command code byte.
        :param data: Array containing the low and high data bytes to be sent to the slave. data[0] will be considered the data_byte_low data[1] will be considered the data_byte_high
        
        :raises MCP2221_Error: raises an exception when DLL error code occurs
        '''
        char_array = ctypes.c_ubyte * 2
        ret = self.Mcp2221_SmbusWriteWord(ctypes.c_void_p(handle), ctypes.c_ubyte(slaveAddress), ctypes.c_ubyte(use7bitAddress), ctypes.c_ubyte(usePec), ctypes.c_ubyte(command), char_array.from_buffer(data))
        if ret != 0:
            raise MCP2221_Error(ret)

