import serial
try:
    s = serial.Serial('COM22', 460800, timeout=0.001)
    print('opened')
    s.close()
except Exception as e:
    import traceback; traceback.print_exc()
    print('error', e)