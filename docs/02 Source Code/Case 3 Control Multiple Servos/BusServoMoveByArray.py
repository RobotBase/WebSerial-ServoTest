from ServoControl import setMoreBusServoMove
import time

if __name__ == '__main__': 
    while True:
        servos = [1, 1000, 4, 300]
        setMoreBusServoMove(servos, 2, 1000)
        time.sleep(2)
        servos = [1, 0, 4, 700]
        setMoreBusServoMove(servos, 2, 1000)
        time.sleep(2)
    
