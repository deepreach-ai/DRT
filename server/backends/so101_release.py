import scservo_sdk as scs
  
TORQUE_ENABLE_ADDR = 40
port = scs.PortHandler('/dev/ttyACM0') # Adjust the port name as needed; leader: /dev/ttyACM0, follower: /dev/ttyACM1
packet = scs.PacketHandler(0)
  
port.openPort()
port.setBaudRate(1000000)
  
for motor_id in range(1, 7):
    result, error = packet.write1ByteTxRx(port, motor_id, TORQUE_ENABLE_ADDR, 0)
    print(f'Motor {motor_id}: result={result}, error={error}')
  
port.closePort()
print('Done.')