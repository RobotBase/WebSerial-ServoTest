from ServoControl import setBusServoMove
import time
if __name__ == '__main__': 
    while True:
        setBusServoMove(1, 0, 1200)
        time.sleep(2)
        setBusServoMove(1, 1000, 1200)
        time.sleep(2)
        setBusServoMove(1, 0, 2000)
        time.sleep(4)
        setBusServoMove(1, 1000, 2000)
        time.sleep(4)
    
