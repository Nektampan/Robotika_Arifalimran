from controller import Robot

TIME_STEP = 64
MAX_SPEED = 6.28

robot = Robot()

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

leftMotor.setVelocity(0.5 * MAX_SPEED)  # Roda kiri lebih lambat
rightMotor.setVelocity(MAX_SPEED)       # Roda kanan lebih cepat

while robot.step(TIME_STEP) != -1:
    pass