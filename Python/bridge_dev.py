"""
Python Bridge Between STM and FactoryIO
"""

import os
from time import sleep
import pathlib
import sys
from threading import Thread, RLock
import serial
import clr

clr.AddReference(str(pathlib.Path().absolute()) + "\\lib\\EngineIO.dll")
import EngineIO


# Use to lock the thread
lock = RLock()


class EngineIOStates(Thread):
    """
    A class used to monitor sensors and actuators states in FactoryIO

    """

    def __init__(self, ser, rate, verbose):
        # Initializing Thread
        Thread.__init__(self)
        self._running = True

        # Creating lists to store sensors and actuators
        self.sensors = []
        self.actuators = []

        # Serial Connection
        self.ser = ser

        # Other parameters
        self.verbose = verbose
        self.rate = rate

        # Linking events to callback function
        EngineIO.MemoryMap.Instance.OutputsNameChanged += (
            EngineIO.MemoriesChangedEventHandler(self.__instance_changed)
        )
        EngineIO.MemoryMap.Instance.OutputsValueChanged += (
            EngineIO.MemoriesChangedEventHandler(self.__instance_changed)
        )
        EngineIO.MemoryMap.Instance.InputsNameChanged += (
            EngineIO.MemoriesChangedEventHandler(self.__instance_changed)
        )
        EngineIO.MemoryMap.Instance.InputsValueChanged += (
            EngineIO.MemoriesChangedEventHandler(self.__instance_changed)
        )

    def __instance_changed(self, _, value):
        """
        This function is called whenever an event pops up
        and updates the sensors and actuators lists.
        """

        # Lock the thread
        with lock:
            # Binary IO
            for mem in value.MemoriesBit:
                # Update names and values of sensors and actuators
                if mem.HasName:
                    # Sensors
                    if mem.MemoryType == EngineIO.MemoryType.Input:
                        # Update if already in list
                        if mem in self.sensors:
                            for sensor in self.sensors:
                                if sensor.Address == mem.Address:
                                    sensor = EngineIO.MemoryMap.Instance.GetBit(
                                        mem.Address, EngineIO.MemoryType.Input
                                    )
                        # Add if not in list
                        else:
                            self.sensors.append(mem)
                    # Actuators
                    elif mem.MemoryType == EngineIO.MemoryType.Output:
                        # Update if already in list
                        if mem in self.actuators:
                            for actuator in self.actuators:
                                if actuator.Address == mem.Address:
                                    actuator = EngineIO.MemoryMap.Instance.GetBit(
                                        mem.Address, EngineIO.MemoryType.Output
                                    )
                        # Add if not in list
                        else:
                            self.actuators.append(mem)
                    else:
                        # Not implemented
                        # EngineIO.MemoryType == EngineIO.MemoryType.Memory
                        pass

                # Clearing items in the list
                else:
                    # Sensors
                    if mem.MemoryType == EngineIO.MemoryType.Input:
                        for sensor in self.sensors:
                            if sensor.Address == mem.Address:
                                self.sensors.remove(sensor)
                    # Actuators
                    elif mem.MemoryType == EngineIO.MemoryType.Output:
                        for actuator in self.actuators:
                            if actuator.Address == mem.Address:
                                self.actuators.remove(actuator)
                    else:
                        # Not implemented
                        # EngineIO.MemoryType == EngineIO.MemoryType.Memory
                        pass

            # Send states via Serial Port
            self.send_states()

    def send_states(self):
        """
        This function sends the updated sensors states via serial port
        """

        # Clear Console
        print("\033[3;0H", end="")

        print("UPDATED SENSORS:")
        # SENSORS UPDATE
        # CRC and Data Bytes for Sensors
        data_byte = 0x00000000
        crc = 0x00
        # Only the first 28 addresses
        for index, sensor in enumerate(self.sensors[0:27]):
            # Print value if verbose
            if sensor.Address < 28 and self.verbose:
                if (index + 1) % 5:
                    print(
                        f"\033[0;93;40m{sensor.Address:02d}\033[0;37;40m",
                        f"\033[0;37;42m{str(sensor.Value)} \033[0;37;40m"
                        if sensor.Value
                        else f"\033[0;37;41m{str(sensor.Value)}\033[0;37;40m",
                        sep=": ",
                        end="    ",
                    )
                else:
                    print(
                        f"\033[0;93;40m{sensor.Address:02d}\033[0;37;40m",
                        f"\033[0;37;42m{str(sensor.Value)} \033[0;37;40m"
                        if sensor.Value
                        else f"\033[0;37;41m{str(sensor.Value)}\033[0;37;40m",
                        sep=": ",
                        end="\n",
                    )

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

        # Separating data into 4 bytes
        data_4_byte = ((data_byte & 0x7F000000) >> 24).to_bytes(1, "big")
        data_3_byte = ((data_byte & 0x007F0000) >> 16).to_bytes(1, "big")
        data_2_byte = ((data_byte & 0x00007F00) >> 8).to_bytes(1, "big")
        data_1_byte = (data_byte & 0x0000007F).to_bytes(1, "big")
        msg = (
            b"\xA8"
            + data_1_byte
            + data_2_byte
            + data_3_byte
            + data_4_byte
            + ((crc << 4) & 0xF0).to_bytes(1, "big")
            + b"\x0A"
        )

        # Sending states to serial port
        try:
            self.ser.write(msg)
        except TimeoutError:
            print("Cannot send message to serial port...")

        # Print sent message if verbose
        if self.verbose:
            crc_string = str(crc.to_bytes(1, "big").hex())
            print(
                "\nSOF\tDATA1\tDATA2\tDATA3\tDATA4\tCRC\t'\\n'\n"
                f"0xA8\t"
                f"0x{str(data_1_byte.hex())}\t"
                f"0x{str(data_2_byte.hex())}\t"
                f"0x{str(data_3_byte.hex())}\t"
                f"0x{str(data_4_byte.hex())}\t"
                f"0x{crc_string}\t"
                f"0x0A\n"
            )

        if self.verbose:
            print("\nUPDATED ACTUATORS:")

        # ACTUATORS UPDATE
        # CRC and Data Bytes for Actuators
        data_byte = 0x00000000
        crc = 0x00
        # Only the first 28 addresses
        for index, actuator in enumerate(self.actuators[0:27]):
            # Print value if verbose
            if actuator.Address < 28 and self.verbose:
                if (index + 1) % 5:
                    print(
                        f"\033[0;96;40m{actuator.Address:02d}\033[0;37;40m",
                        f"\033[0;37;42m{str(actuator.Value)} \033[0;37;40m"
                        if actuator.Value
                        else f"\033[0;37;41m{str(actuator.Value)}\033[0;37;40m",
                        sep=": ",
                        end="    ",
                    )
                else:
                    print(
                        f"\033[0;96;40m{actuator.Address:02d}\033[0;37;40m",
                        f"\033[0;37;42m{str(actuator.Value)} \033[0;37;40m"
                        if actuator.Value
                        else f"\033[0;37;41m{str(actuator.Value)}\033[0;37;40m",
                        sep=": ",
                        end="\n",
                    )
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
        data_4_byte = ((data_byte & 0x7F000000) >> 24).to_bytes(1, "big")
        data_3_byte = ((data_byte & 0x007F0000) >> 16).to_bytes(1, "big")
        data_2_byte = ((data_byte & 0x00007F00) >> 8).to_bytes(1, "big")
        data_1_byte = (data_byte & 0x0000007F).to_bytes(1, "big")
        msg = (
            b"\xAD"
            + data_1_byte
            + data_2_byte
            + data_3_byte
            + data_4_byte
            + ((crc << 4) & 0xF0).to_bytes(1, "big")
            + b"\x0A"
        )

        # Sending states to serial port
        try:
            self.ser.write(msg)
        except TimeoutError:
            print("Cannot send message to serial port...")

        # Print sent message if verbose
        if self.verbose:
            crc_string = str(crc.to_bytes(1, "big").hex())
            print(
                "\nSOF\tDATA1\tDATA2\tDATA3\tDATA4\tCRC\t'\\n'\n"
                f"0xAD\t"
                f"0x{str(data_1_byte.hex())}\t"
                f"0x{str(data_2_byte.hex())}\t"
                f"0x{str(data_3_byte.hex())}\t"
                f"0x{str(data_4_byte.hex())}\t"
                f"0x{crc_string}\t"
                f"0x0A\n"
            )

    def run(self):
        """This function runs the EngineIO_States thread"""
        while self._running:
            EngineIO.MemoryMap.Instance.Update()
            sleep(self.rate)  # FactoryIO update rate default value is 16ms

    def close(self):
        self._running = False


class EngineIOCommands(Thread):
    """
    A class waiting for commands on serial port to send them to FactoryIO
    """

    def __init__(self, ser, stateThread, n_console_lines=5):
        # Initializing Thread
        Thread.__init__(self, target=self.run)
        self._running = True
        self.state_thread = stateThread

        # Serial Connection
        self.ser = ser

        # Console lines
        self._line_counter = 0
        self._saved_lines = []
        self._max_lines = n_console_lines

    def __send_command(self, cmd):
        """
        A function to send commands to FactoryIO sdk
        (called whenever a message is received by the thread)
        """
        # Lock the thread
        with lock:
            # Verifiy if command
            if cmd[0] == 0xAD:
                # Separate data and check bytes
                data1_byte = cmd[1]
                data2_byte = cmd[2]
                data3_byte = cmd[3]
                data4_byte = cmd[4]
                _ = cmd[5]  # Ignore CRC
                # Send data 1 to FactoryIO (Address 0 to 6)
                for i in range(8):
                    actuator = EngineIO.MemoryMap.Instance.GetBit(
                        i, EngineIO.MemoryType.Output
                    )
                    actuator.set_Value(bool((data1_byte & (1 << i)) >> i))
                    EngineIO.MemoryMap.Instance.Update()
                # Send data 2 to FactoryIO (Address 7 to 13)
                for i in range(8):
                    actuator = EngineIO.MemoryMap.Instance.GetBit(
                        i + 7, EngineIO.MemoryType.Output
                    )
                    actuator.set_Value(bool((data2_byte & (1 << i)) >> i))
                    EngineIO.MemoryMap.Instance.Update()
                # Send data 3 to FactoryIO (Address 14 to 20)
                for i in range(8):
                    actuator = EngineIO.MemoryMap.Instance.GetBit(
                        i + 14, EngineIO.MemoryType.Output
                    )
                    actuator.set_Value(bool((data3_byte & (1 << i)) >> i))
                    EngineIO.MemoryMap.Instance.Update()
                # Send data 4 to FactoryIO (Address 21 to 27)
                for i in range(8):
                    actuator = EngineIO.MemoryMap.Instance.GetBit(
                        i + 21, EngineIO.MemoryType.Output
                    )
                    actuator.set_Value(bool((data4_byte & (1 << i)) >> i))
                    EngineIO.MemoryMap.Instance.Update()

                EngineIOStates.send_states(self.state_thread)

            # Force Update States
            elif cmd[0] == 0xA3:
                EngineIOStates.send_states(self.state_thread)
            # Passthrough for serial port to console
            else:
                print("\033[0;97;44m", end="")  # Set color to blue
                if len(self._saved_lines) == self._max_lines:
                    self._saved_lines.pop(0)  # Remove first line
                self._saved_lines.append(cmd)  # Add new line
                for line in self._saved_lines:
                    print("\033[2K\r", end="")  # Clear line
                    print(f">> {line}")  # Print line
                print("\033[0;97;40m", end="")  # Set color to black

    def run(self):
        """This function runs the EngineIO_Commands thread"""
        while self._running:
            try:

                msg = self.ser.readline()

                if msg is not None and len(msg) > 0 and len(msg) < 6 and msg[0] > 127:
                    msg += self.ser.readline()

                if msg is not None and len(msg) > 0:
                    self.__send_command(msg)

            # If time is out
            except TimeoutError:
                print("No data sent for " + str(self.ser.timeout) + "seconds...")
            except serial.SerialException as err:
                print(err)
            except Exception as err:
                print("COMMANDS THREAD EXCEPTION")
                print(err)
                print(msg.hex())
                print("END OF COMMANDS THREAD EXCEPTION")

    def close(self):
        """This functions stops the thread from staying in its while loop"""
        self._running = False


class EngineIOController:
    def __init__(
        self,
        port="COM5",
        baudrate=115200,
        timeout=0.5,
        rate=0.016,
        verbose=False,
        n_console_lines=5,
    ):

        # Serial parameters
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        # Other parameters
        self.verbose = verbose
        self.rate = rate  # FactoryIO default update rate is 16ms
        self._running = True
        self.max_lines = n_console_lines

        # Initializing serial port
        try:
            self.ser = serial.Serial(
                port=self.port, baudrate=self.baudrate, timeout=self.timeout
            )
        except serial.SerialException as err:
            print(f"Error: {err}")
            sys.exit(1)  # Exit program

        print(
            f"Successfully connected to UART "
            f"on {self.port} at {self.baudrate} bauds\n"
        )

        # Creating Threads
        self.state_thread = EngineIOStates(
            ser=self.ser, rate=self.rate, verbose=self.verbose
        )
        self.commands_thread = EngineIOCommands(
            ser=self.ser,
            stateThread=self.state_thread,
            n_console_lines=self.max_lines,
        )

        # Running
        try:
            # Starting threads
            self.state_thread.daemon = True
            self.commands_thread.deamon = True
            self.state_thread.start()
            self.commands_thread.start()

            # Sleep the main thread
            while self._running:
                sleep(0.5)

        # If Ctrl-C is pressed
        except KeyboardInterrupt:
            self._running = False
            self.close()

    def close(self):
        try:
            # Terminating threads when over (when using Ctrl-C)
            self.state_thread.close()
            self.commands_thread.close()
            self.state_thread.join()
            self.commands_thread.join()
            del self.state_thread
            del self.commands_thread

            # Closing serial port
            self.ser.close()

            # SystemExit
            sys.exit(1)

        except SystemExit:
            os.system("cls" if os.name == "nt" else "clear")
            print("Closing Controller.py after KeyboardInterrupt...")


if __name__ == "__main__":
    os.system("")
    print("\033[0;37;40m", end="")
    os.system("cls" if os.name == "nt" else "clear")
    EngineIOController(
        port="COM37",
        baudrate=9600,
        timeout=0.5,
        rate=0.016,
        verbose=True,
        n_console_lines=10,
    )
