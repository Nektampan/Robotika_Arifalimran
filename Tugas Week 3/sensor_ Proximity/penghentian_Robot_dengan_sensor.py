from controller import Robot

TIME_STEP = 64
MAX_SPEED = 6.28
robot = Robot()

leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")

leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))

rightSensor = robot.getDevice("ps0")
leftSensor = robot.getDevice("ps7")

rightSensor.enable(TIME_STEP)
leftSensor.enable(TIME_STEP)

def move_robot():
    leftMotor.setVelocity(MAX_SPEED)
    rightMotor.setVelocity(MAX_SPEED)
    
def stop_robot():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)

while robot.step(TIME_STEP) != -1:
    right_sensor = rightSensor.getValue()
    left_sensor = leftSensor.getValue()
    print("Right Sensor Value:", right_sensor)
    print("Left Sensor Value:", left_sensor)
    
    if right_sensor > 76 or left_sensor > 76:
        print("Stop Robot")
        stop_robot()    
    else:
        move_robot()



   