"""
Python Bridge Between STM and FactoryIO

Version: 1.1.3

"""

import serial
from time import sleep
from threading import Thread, RLock, Event
import pathlib
import sys
import clr

clr.AddReference(str(pathlib.Path().absolute()) + "\\lib\\EngineIO.dll")
from EngineIO import *


# Use to lock the thread
lock = RLock()


class EngineIO_States(Thread):
    """
    A class used to monitor sensors and actuators states in FactoryIO

    """

    def __init__(self, ser, rate, verbose):
        # Initializing Thread
        Thread.__init__(self)
        self._running = True

        # Creating lists to store sensors and actuators
        self.sensors = list()
        self.actuators = list()

        # Serial Connection
        self.ser = ser

        # Other parameters
        self.verbose = verbose
        self.rate = rate

        # Linking events to callback function
        MemoryMap.Instance.OutputsNameChanged += MemoriesChangedEventHandler(
            self.__Instance_Changed)
        MemoryMap.Instance.OutputsValueChanged += MemoriesChangedEventHandler(
            self.__Instance_Changed)
        MemoryMap.Instance.InputsNameChanged += MemoriesChangedEventHandler(
            self.__Instance_Changed)
        MemoryMap.Instance.InputsValueChanged += MemoriesChangedEventHandler(
            self.__Instance_Changed)

    def __Instance_Changed(self, sender, value):
        """This function is called whenever an event pops up and updates the sensors and actuators lists."""

        # Lock the thread
        with lock:
            # Show event sender and value if verbose
            if self.verbose:
                print('\n{} - {}'.format(sender, value))

            # Binary IO
            for mem in value.MemoriesBit:
                # Update names and values of sensors and actuators
                if mem.HasName:
                    # Sensors
                    if mem.MemoryType == MemoryType.Input:
                        # Update if already in list
                        if mem in self.sensors:
                            for sensor in self.sensors:
                                if sensor.Address == mem.Address:
                                    if self.verbose:
                                        print("Changed sensor {} to value {}".format(str(sensor.Address), str(sensor.Value)))
                                    sensor = MemoryMap.Instance.GetBit(mem.Address, MemoryType.Input)
                        # Add if not in list
                        else:
                            self.sensors.append(mem)
                    # Actuators
                    elif mem.MemoryType == MemoryType.Output:
                        # Update if already in list
                        if mem in self.actuators:
                            for actuator in self.actuators:
                                if actuator.Address == mem.Address:
                                    actuator = MemoryMap.Instance.GetBit(mem.Address, MemoryType.Output)
                        # Add if not in list
                        else:
                            self.actuators.append(mem)
                    else:
                        # Not implemented
                        # MemoryType == MemoryType.Memory
                        pass

                    # Print updated items if verbose
                    if self.verbose:
                        if mem.MemoryType == MemoryType.Memory:
                            memType = "Memory"
                        elif mem.MemoryType == MemoryType.Input:
                            memType = "Sensor"
                        elif mem.MemoryType == MemoryType.Output:
                            memType = "Actuator"
                        print("{0} Bit({1}) name changed to: {2}".format(
                            memType, str(mem.Address), str(mem.Name)))

                # Clearing items in the list
                else:
                    # Sensors
                    if mem.MemoryType == MemoryType.Input:
                        for sensor in self.sensors:
                            if sensor.Address == mem.Address:
                                self.sensors.remove(sensor)
                    # Actuators
                    elif mem.MemoryType == MemoryType.Output:
                        for actuator in self.actuators:
                            if actuator.Address == mem.Address:
                                self.actuators.remove(actuator)
                    else:
                        # Not implemented
                        # MemoryType == MemoryType.Memory
                        pass

                    # Print cleared items if verbose
                    if not self.verbose:
                        if mem.MemoryType == 0:
                            memType = "Memory"
                        elif mem.MemoryType == 1:
                            memType = "Sensor"
                        elif mem.MemoryType == 2:
                            memType = "Actuator"
                        print("{0} Bit({1}) name cleared".format(
                            memType, str(mem.Address)))

            if self.verbose: print('')
            # Send states via Serial Port
            self._send_states()

    def _send_states(self):
        """This function sends the sensors states changes via serial port"""
        # SENSORS UPDATE
        # CRC and Data Bytes for Sensors
        data_byte = 0x00000000
        crc = 0x00
        # Only the first 28 addresses
        for sensor in self.sensors[0:27]:
            # Print value if verbose
            if sensor.Address < 28 and self.verbose:
                if (sensor.Address+1)%5:
                    print('SEN@' + str(sensor.Address), str(sensor.Value), sep=':', end='\t')
                else:
                    print('SEN@' + str(sensor.Address), str(sensor.Value), sep=':', end='\n')

            # SENSORS
            if sensor.Value:
                if sensor.Address <= 6:
                    data_byte = data_byte + (1 << int(sensor.Address))
                    crc += 1
                elif sensor.Address <= 13:
                    data_byte = data_byte + (1 << (int(sensor.Address) + 1))
                    crc += 1
                elif sensor.Address <= 20:
                    data_byte = data_byte + (1 << (int(sensor.Address) + 2))
                    crc += 1
                elif sensor.Address <= 27:
                    data_byte = data_byte + (1 << (int(sensor.Address) + 3))
                    crc += 1

        if self.verbose: print('')

        # Separating data into 4 bytes
        data_4_byte = ((data_byte & 0x7F000000) >> 24).to_bytes(1, 'big')
        data_3_byte = ((data_byte & 0x007F0000) >> 16).to_bytes(1, 'big')
        data_2_byte = ((data_byte & 0x00007F00) >> 8).to_bytes(1, 'big')
        data_1_byte = (data_byte & 0x0000007F).to_bytes(1, 'big')
        msg = b'\xA8' + data_1_byte + data_2_byte + data_3_byte + data_4_byte + ((crc << 4) & 0xF0).to_bytes(1, 'big') + b'\x0A'

        # Sending states to serial port
        try:
            self.ser.write(msg)
        except TimeoutError:
            print("Cannot send message to serial port...")

        # Print sent message if verbose
        if self.verbose:
            print("\nMessage reçu:\nSOF\tDATA1\tDATA2\tDATA3\tDATA4\tCRC\t\'\\n\'\n{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format('0x' + str(b'\xA8'.hex()), '0x' + str(data_1_byte.hex()), '0x' + str(data_2_byte.hex()), '0x' + str(data_3_byte.hex()), '0x' + str(data_4_byte.hex()), '0x' + str(crc.to_bytes(1, 'big').hex()), '0x' + str(b'\x0A'.hex())))

        if self.verbose: print('--------')

        # ACTUATORS UPDATE
        # CRC and Data Bytes for Actuators
        data_byte = 0x00000000
        crc = 0x00
        # Only the first 28 addresses
        for actuator in self.actuators[0:27]:
            # Print value if verbose
            if actuator.Address < 28 and self.verbose:
                if (actuator.Address+1)%5:
                    print('ACT@' + str(actuator.Address), str(actuator.Value), sep=':', end='\t')
                else:
                    print('ACT@' + str(actuator.Address), str(actuator.Value), sep=':', end='\n')
            if actuator.Value:
                if actuator.Address <= 6:
                    data_byte = data_byte + (1 << int(actuator.Address))
                    crc += 1
                elif actuator.Address <= 13:
                    data_byte = data_byte + (1 << (int(actuator.Address) + 1))
                    crc += 1
                elif actuator.Address <= 20:
                    data_byte = data_byte + (1 << (int(actuator.Address) + 2))
                    crc += 1
                elif actuator.Address <= 27:
                    data_byte = data_byte + (1 << (int(actuator.Address) + 3))
                    crc += 1

        # Separating data into 4 bytes
        data_4_byte = ((data_byte & 0x7F000000) >> 24).to_bytes(1, 'big')
        data_3_byte = ((data_byte & 0x007F0000) >> 16).to_bytes(1, 'big')
        data_2_byte = ((data_byte & 0x00007F00) >> 8).to_bytes(1, 'big')
        data_1_byte = (data_byte & 0x0000007F).to_bytes(1, 'big')
        msg = b'\xAD' + data_1_byte + data_2_byte + data_3_byte + data_4_byte + ((crc << 4) & 0xF0).to_bytes(1, 'big') + b'\x0A'

        # Sending states to serial port
        try:
            self.ser.write(msg)
        except TimeoutError:
            print("Cannot send message to serial port...")

        # Print sent message if verbose
        if self.verbose:
            print("\nMessage envoyé:\nSOF\tDATA1\tDATA2\tDATA3\tDATA4\tCRC\t\'\\n\'\n{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format('0x' + str(b'\xAD'.hex()), '0x' + str(data_1_byte.hex()), '0x' + str(data_2_byte.hex()), '0x' + str(data_3_byte.hex()), '0x' + str(data_4_byte.hex()), '0x' + str(crc.to_bytes(1, 'big').hex()), '0x' + str(b'\x0A'.hex())))

        if self.verbose: print('--------')

    def run(self):
        """This function runs the EngineIO_States thread"""
        while self._running:
            MemoryMap.Instance.Update()
            sleep(self.rate)  # FactoryIO update rate default value is 16ms

    def close(self):
        self._running = False


class EngineIO_Commands(Thread):
    """
    A class waiting for commands on serial port to send them to FactoryIO
    """

    def __init__(self, ser, rate, verbose, stateThread):
        # Initializing Thread
        Thread.__init__(self, target=self.run)
        self._running = True
        self.stateThread = stateThread

        # Serial Connection
        self.ser = ser

    def __send_command(self, cmd):
        """A function to send commands to FactoryIO sdk (called whenever a message is received by the thread)"""
        # Lock the thread
        with lock:
            # Verifiy if command
            if cmd[0] == 0xAD:
                # Separate data and check bytes
                data1_byte = cmd[1]
                data2_byte = cmd[2]
                data3_byte = cmd[3]
                data4_byte = cmd[4]
                crc_byte = cmd[5]
                # Send data 1 to FactoryIO (Address 0 to 6)
                for i in range(8):
                    actuator = MemoryMap.Instance.GetBit(i, MemoryType.Output)
                    actuator.Value = (data1_byte & (1 << i)) >> i
                    MemoryMap.Instance.Update()
                # Send data 2 to FactoryIO (Address 7 to 13)
                for i in range(8):
                    actuator = MemoryMap.Instance.GetBit(i+7, MemoryType.Output)
                    actuator.Value = (data2_byte & (1 << i)) >> i
                    MemoryMap.Instance.Update()
                # Send data 3 to FactoryIO (Address 14 to 20)
                for i in range(8):
                    actuator = MemoryMap.Instance.GetBit(i+14, MemoryType.Output)
                    actuator.Value = (data3_byte & (1 << i)) >> i
                    MemoryMap.Instance.Update()
                # Send data 4 to FactoryIO (Address 21 to 27)
                for i in range(8):
                    actuator = MemoryMap.Instance.GetBit(i+21, MemoryType.Output)
                    actuator.Value = (data4_byte & (1 << i)) >> i
                    MemoryMap.Instance.Update()


                EngineIO_States._send_states(self.stateThread)

            # Force Update States
            elif cmd[0] == 0xA3:
                EngineIO_States._send_states(self.stateThread)
            # Passthrough for serial port to console
            else:
                print('>> ' + str(cmd))

    def run(self):
        """This function runs the EngineIO_Commands thread"""
        while self._running:
            try:

                msg = self.ser.readline()

                if msg != None and len(msg) > 0 and len(msg) < 6 and msg[0] > 127:
                    msg += self.ser.readline()

                if msg != None and len(msg) > 0:
                    self.__send_command(msg)

            # If time is out
            except TimeoutError:
                print('No data sent for ' + str(elf.timeout) + 'seconds...')
            except Exception as err:
                print("COMMANDS THREAD EXCEPTION")
                print(err)
                print(msg.hex())
                print("END OF COMMANDS THREAD EXCEPTION")

    def close(self):
        """This functions stops the thread from staying in its while loop"""
        self._running = False


class EngineIO_Controller():

    def __init__(self, port='COM5', baudrate=115200, timeout=.5, rate=0.016, verbose=False):

        # Serial parameters
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        # Other parameters
        self.verbose = verbose
        self.rate = rate  # FactoryIO default update rate is 16ms
        self._running = True

        # Initializing serial port
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
        except Exception:
            print("error")

        # Creating Threads
        self.stateThread = EngineIO_States(ser=self.ser,rate=self.rate, verbose=self.verbose)
        self.commandsThread = EngineIO_Commands(ser=self.ser, rate=self.rate, verbose=self.verbose, stateThread=self.stateThread)

        # Running
        try:
            # Starting threads
            self.stateThread.setDaemon(True)
            self.commandsThread.setDaemon(True)
            self.stateThread.start()
            self.commandsThread.start()

            # Sleep the main thread
            while self._running:
                sleep(.5)

        # If Ctrl-C is pressed
        except KeyboardInterrupt:
            self._running = False
            self.close()

    def close(self):
        try:
            # Terminating threads when over (when using Ctrl-C)
            self.stateThread.close()
            self.commandsThread.close()
            self.stateThread.join()
            self.commandsThread.join()
            del self.stateThread
            del self.commandsThread

            # Closing serial port
            self.ser.close()

            # SystemExit
            sys.exit(1)

        except SystemExit:
            print("Closing Controller.py after KeyboardInterrupt...")

        except Exception as err:
            print("EXCEPTION")
            print(err)
            print("END OF EXCEPTION")


if __name__ == "__main__":
    EngineIO_Controller(port='COM41', baudrate=9600, timeout=.5, rate=0.016,  verbose=True)
