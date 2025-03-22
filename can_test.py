import can

bus = can.interface.Bus(channel='can0', interface='socketcan', fd=True) #, bitrate=1_000_000, dbitrate=8_000_000)

msg = can.Message(arbitration_id=0x321, data=[0xFF, 0xAA], is_extended_id=False, is_fd=True) #, bitrate_switch=True)
bus.send(msg)

bus.shutdown()