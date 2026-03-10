from ServoControl import setBusServoMove
import time

if __name__ == '__main__': 
    deviation = 30
    setBusServoMove(1, 500, 800)
    time.sleep(2)
    setBusServoMove(1, 500+deviation, 800)
    while True:
        time.sleep(1)

    
