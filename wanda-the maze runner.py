from controller import Robot, DistanceSensor, Motor

#-------------------------------------------------------
# Initialize variables

speed = 6.28

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())  

# distance sensors
ps = []
psNames = ['ps4','ps6','ps7']
for i in range(3):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

# motors    
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)


#-------------------------------------------------------
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    psValues = []
    for i in range(3):
        psValues.append(ps[i].getValue())
        
    leftSpeed  =speed
    rightSpeed =speed

    left_obstacle = psValues[0]>80.0
    left_corner=psValues[1]>80.0  
    front_obstacle=psValues[2]>80.0
    # detect obstacles
    if  front_obstacle:
        # turn right        
        leftSpeed  =speed
        rightSpeed =-speed/8
    else:
         if left_obstacle:
             leftSpeed  =speed
             rightSpeed =speed
         elif left_corner:
             leftSpeed  =speed
             rightSpeed =-speed/8
 
                    
  
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
