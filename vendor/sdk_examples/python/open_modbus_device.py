"""Modbus connection example.

Steps:
    1. Create the serial control instance.
    2. Connect to the selected hand.
    3. Query hand type.
    4. Initialize motor position.
    5. Query online status.
    6. Close the Modbus serial port.
"""

import argparse
import sys
from pxdex.dh15 import DexH15Control


# Manual connection usage reference example
if __name__ == "__main__":
    """
    Steps:
        1. Create serial control instance [required]                              
        2. Open Modbus serial port [required]                                
        3. Scan device addresses [optional]
        4. Initialize slave device address [required]                                
        5. Initialize motor position [required for position control, call once after power-on and connection] 
        6. Other operations, e.g., connection check [optional]                              
        7. Close Modbus serial port [required]
    """

    # 1. Create serial control instance
    hand_control = DexH15Control()
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Modbus serial port, default: /dev/ttyUSB0")
    parser.add_argument("--addr", type=lambda value: int(value, 0), default=None,
                        help="Optional Modbus slave address. If omitted, connectModbusAuto is used.")
    args = parser.parse_args()

    if args.addr is None:
        connect_ret, slave_address = hand_control.connectModbusAuto(args.port)
        if not connect_ret:
            print("Please check if the dexterous hand is connected to the Modbus serial port")
            sys.exit(1)
        print("Auto connect Modbus success")
    else:
        slave_address = args.addr
        if not hand_control.openModbusDevice(args.port, 4000000):
            print("Please check if the Modbus serial device is connected")
            sys.exit(1)
        print("Open Modbus serial port success")
        if hand_control.initModbusDevice(slave_address) != 1:
            print("Please check if the dexterous hand is connected to the Modbus serial device")
            hand_control.disconnectModbus()
            sys.exit(1)
        print("Init Modbus slave success")

    hand_type = hand_control.getHandType(slave_address)
    print(f"Modbus device address: 0x{slave_address:02x}")
    print(f"Hand type: {str(hand_type).split('.')[-1]}")
        
    # 5. Initialize motor position [required for position control, call once after power-on and connection]
    if hand_control.initMotorPosition(slave_address) != 1:
        print("Init motor position failed")
        sys.exit(1)
    print("Init motor position success")
        
    # 6. Other operations, e.g., connection check
    is_connected = hand_control.isModbusDeviceConnected(slave_address)
    print(f"Dexterous hand {'online' if is_connected else 'offline'}")
        
    # 7. Close Modbus serial port
    result = hand_control.disconnectModbus()
    print(f"Disconnect: {'success' if result else 'failed'}")
