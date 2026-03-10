from ServoControl import setBusServoMove
import time

if __name__ == '__main__': 
    while True:
        setBusServoMove(1, 0, 1000)
        time.sleep(2)
        setBusServoMove(1, 1000, 1000)
        time.sleep(2)
    
