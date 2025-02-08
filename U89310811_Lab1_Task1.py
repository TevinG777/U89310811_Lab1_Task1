# Import MyRobot Class
from fairis_tools.my_robot import MyRobot
import math

stepCounter = 0
pollingCounter = 0
encoderReading = 0
totalRadtoRemove = 0
stepNumber = 0
lastFuntion = ''


max_linear_velocity = 1.118

# Create the robot instance.
robot = MyRobot()

# Loads the environment from the maze file
maze_file = '../../worlds/Spring25/maze1.xml'
robot.load_environment(maze_file)

# Move robot to a random staring position listed in maze file
robot.move_to_start()

def rotate(desiredHeading, speed, stepNum):
    global stepNumber
    
    if stepNum == stepNumber:
        pass
    else:
        return stepNumber
    
    
    
    # normalize the 0-360 degree heading to -180 to 180 degrees in order to calculate the shortest path
    tmpDesiredHeading = (desiredHeading + 180) % 360 -180
    
    # get the current heading of the robot
    currentHeading = robot.get_compass_reading()
    
    # normalize the current heading to -180 to 180 degrees
    tmpCurrentHeading = (currentHeading + 180) % 360 -180
    
    # Determine if it is faster to turn right or left
    if tmpDesiredHeading - tmpCurrentHeading > 0:
        
        # if the desired heading is greater than the current heading, turn right
        leftSpeed = -speed
        rightSpeed = speed
        robot.set_left_motors_velocity(leftSpeed)
        robot.set_right_motors_velocity(rightSpeed)
        
        # Calculate the angular velocity of the robot in order to determine the time it will take to reach the desired heading
        # Convert the angular velocity to linear velocity
        V_L = robot.wheel_radius * leftSpeed
        V_R = robot.wheel_radius * rightSpeed
        
        # Calculate the angular velocity of the robot
        angularVelocity = (V_R - V_L)/robot.axel_length
        
        # Calculate the time it will take to reach the desired heading (2pi/|angular velocity|) also make sure angular velo is positivie
        estTime = 2*math.pi / math.abs(angularVelocity)
        
        # Call the announce values function to print out the values
        printAnnounceValues(rotate, leftSpeed, rightSpeed, estTime, 0)
        
        # Call the nav print values function to print out the values
        printNavValues(leftSpeed, rightSpeed, 0, robot.experiment_supervisor.getTime())
        
        # if the robot has reached the desired heading, stop the robot and update the step number to move to the next step
        if(robot.get_compass_reading() >= desiredHeading):
            robot.stop()
            
            # print new line to make output look nicer
            print('\n')
            
            stepNumber += 1

    
            return 1
    else:
        # if the desired heading is less than the current heading, turn left
        leftSpeed = speed
        rightSpeed = -speed
        robot.set_left_motors_velocity(leftSpeed)
        robot.set_right_motors_velocity(rightSpeed)
        
        # Calculate the angular velocity of the robot in order to determine the time it will take to reach the desired heading
        # Convert the angular velocity to linear velocity
        V_L = robot.wheel_radius * leftSpeed
        V_R = robot.wheel_radius * rightSpeed
        
        # Calculate the angular velocity of the robot
        angularVelocity = (V_R - V_L)/robot.axel_length
        
        # Calculate the time it will take to reach the desired heading (2pi/|angular velocity|) also make sure angular velo is positivie
        estTime = 2*math.pi / math.abs(angularVelocity)
        
        # Call the announce values function to print out the values
        printAnnounceValues(rotate, leftSpeed, rightSpeed, estTime, 0)
        
        # Call the nav print values function to print out the values
        printNavValues(leftSpeed, rightSpeed, 0, robot.experiment_supervisor.getTime())
        
        # if the robot has reached the desired heading, stop the robot and update the step number to move to the next step
        if(robot.get_compass_reading() <= desiredHeading):
            robot.stop()
            
            # print new line to make output look nicer
            print('\n')
            stepNumber += 1
            
            return 1
        
def moveForward(desiredHeading, startingX, startingY, endingX, endingY, velo, distanceOffset, stepNum):
    
    global encoderReading
    global stepNumber
    
    
    if stepNum == stepNumber:
        pass
    else:
        return stepNumber

    
    # calculate the distance traveled between the points using the distance formula
    xComponent = math.pow((endingX-startingX),2)
    yComponent = math.pow((endingY-startingY),2)
    distance = math.sqrt(xComponent+yComponent)

    
    
    # Grab reading from encoders to determine how far it has traveled
    accumulatedDis = robot.wheel_radius * robot.get_front_right_motor_encoder_reading()
    
    # convert encoder reading to distance
    encoderOffset = robot.wheel_radius * encoderReading
    
    # Only print out the values if the funtion has not yet be completed
    if accumulatedDis < distance + encoderOffset:
        
        # Set the motors to the desired speed
        robot.set_right_motors_velocity(velo)
        robot.set_left_motors_velocity(velo)
        
    # Calculate the estimated time it will take to reach the desired distance
    estTime = distance/(velo*robot.wheel_radius)
        
    # print out the values
    printAnnounceValues(moveForward, velo, velo, estTime, distance)
    printNavValues(velo, velo, distance, robot.experiment_supervisor.getTime())
        
        
    # if the robot has traveled the distance, stop the robot
    if(accumulatedDis > distance+distanceOffset+encoderOffset):
        # Stop the robot and update the encoder readings only on the first time accumulatedDis > distance
        if accumulatedDis < distance+distanceOffset + 0.05 + encoderReading:
            
            # update the step number and encoder reading and stop the robot
            stepNumber += 1
            encoderReading = robot.get_front_right_motor_encoder_reading()
            
            # print new line to make output look nicer
            print('\n')
            
            robot.stop()
            
        return 1
        
def curvedTurn(desiredHeading, radius, rads, direction, maxSpeed, distanceOffset, stepNum):
    global encoderReading
    global pollingCounter 
    global stepNumber
    
    if stepNum == stepNumber:
        pass
    else:
        return stepNumber
    
    # Determine the distance the robot needs to travel to make the turn
    distance = (2 * math.pi * radius)*(rads/(2*math.pi))
    
    
    # Calcuate the angular velocity of the robot
    angularVelocity = (maxSpeed*robot.wheel_radius)/(radius+(robot.axel_length/2))
    
    # if moving left, set the right motor to max speed and if moving right, set the left motor to max speed
    if direction == "left":
        rightDistance = (2 * math.pi * (radius + robot.axel_length/2))*(rads/(2*math.pi))
        VeloRight = maxSpeed
        VeloLeft = angularVelocity * (radius - robot.axel_length/2)
        VeloLeft = VeloLeft/robot.wheel_radius  
        
    else:
        rightDistance = (2 * math.pi * (radius - robot.axel_length/2))*(rads/(2*math.pi))
        VeloLeft = maxSpeed
        VeloRight = angularVelocity * (radius - robot.axel_length/2)
        VeloRight = VeloRight/robot.wheel_radius

        
    # Grab reading from encoders to determine how far it has traveled
    accumulatedDis = robot.wheel_radius * robot.get_front_right_motor_encoder_reading()
    
    # convert encoder reading to distance
    encoderOffset = robot.wheel_radius * encoderReading
    
    
    # Set the motors to calculated speed
    if accumulatedDis < rightDistance + encoderOffset :
        robot.set_left_motors_velocity(VeloLeft)
        robot.set_right_motors_velocity(VeloRight)

    
    # if the robot has traveled the distance, stop the robot
    if(accumulatedDis > rightDistance+encoderOffset + distanceOffset):
        # Stop the robot and update the encoder readings only on the first time accumulatedDis > distance
        if accumulatedDis < distance + encoderReading + distanceOffset:

            # update the step number and encoder reading and stop the robot
            stepNumber += 1
            encoderReading = robot.get_front_right_motor_encoder_reading()

            robot.stop()
        return 1
        
def printNavValues(VeloLeft, VeloRight, distance, time):
    print('V_li = {%0.1f}, V_ri = {%0.1f}, D = {%0.1f}, T = {%0.1f}' % (VeloLeft, VeloRight, distance, time))
    
def printAnnounceValues(func,VeloLeft, VeloRight, estimatedTime, distance):
    global lastFuntion
    
    if lastFuntion == func:
        return
    print('---------------------------------')
    print("VeloLeft: ", VeloLeft)
    print("VeloRight: ", VeloRight)
    print("Estimated Time: ", estimatedTime)
    print("Distance: ", distance)
    print('---------------------------------')
    print('\n')
    
    # update the last function called to prevent multiple print statements
    lastFuntion = func

    

def callFunction(func, *args):
    global stepNumber
    
    if func == moveForward and args[-1] == stepNumber:
        func(*args)
        
    elif func == rotate and args[-1] == stepNumber:
        func(*args)
        
    elif func == curvedTurn and args[-1] == stepNumber:
        func(*args)
    
# Main Control Loop for Robot
while robot.experiment_supervisor.step(robot.timestep) != -1:
    pass
    

    

    robot.experiment_supervisor.getTime()
    
    