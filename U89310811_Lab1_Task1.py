# Import MyRobot Class
from fairis_tools.my_robot import MyRobot
import math

# Global Variables
pollingCounter = 0
encoderReading = 0
stepNumber = 0
lastFuntion = ''
timeAccumulator = 0


max_linear_velocity = 1.118

# Create the robot instance.
robot = MyRobot()

# Loads the environment from the maze file
maze_file = '../../worlds/Spring25/maze1.xml'
robot.load_environment(maze_file)

# Move robot to a random staring position listed in maze file
robot.move_to_start()

def rotate(desiredHeading, speed, point1, point2, adjust, stepNum):
    global stepNumber
    global timeAccumulator
    
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
        angularVelocity = (V_L - V_R)/robot.axel_length
        
        # Calcuate the distance the robot needs to turn by using the formula (desiredHeading - currentHeading) and convert to radians
        totalRadtoRemove = math.radians(tmpDesiredHeading - tmpCurrentHeading) 
        
        # Calculate the time it will take to reach the desired heading (2pi/|angular velocity|) also make sure angular velo is positivie
        estTime = totalRadtoRemove / math.fabs(angularVelocity)
        
        # Call the announce values function to print out the values
        printAnnounceValues(rotate, leftSpeed, rightSpeed, estTime, point1, point2, 0, adjust, desiredHeading)
        
        # Call the nav print values function to print out the values
        printNavValues(leftSpeed, rightSpeed, 0, robot.experiment_supervisor.getTime())
        
        # if the robot has reached the desired heading, stop the robot and update the step number to move to the next step
        if(robot.get_compass_reading() >= desiredHeading):
            robot.stop()
            
            # print new line to make output look nicer
            print('\n')
            
            # Grab the current time and add to the time accumulator
            timeAccumulator += robot.experiment_supervisor.getTime()
            
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
        
        # Calcuate the distance the robot needs to turn by using the formula (desiredHeading - currentHeading) and convert to radians
        totalRadtoRemove = math.radians(tmpDesiredHeading - tmpCurrentHeading) 
        
        # Calculate the time it will take to reach the desired heading (2pi/|angular velocity|) also make sure angular velo is positivie
        estTime = totalRadtoRemove / math.fabs(angularVelocity)
        
        # Call the announce values function to print out the values
        printAnnounceValues(rotate, leftSpeed, rightSpeed, estTime, point1, point2, 0, adjust, desiredHeading)
        
        # Call the nav print values function to print out the values
        printNavValues(leftSpeed, rightSpeed, 0, robot.experiment_supervisor.getTime())
        
        # if the robot has reached the desired heading, stop the robot and update the step number to move to the next step
        if(robot.get_compass_reading() <= desiredHeading):
            robot.stop()
            
            # print new line to make output look nicer
            print('\n')
            
            # Grab the current time and add to the time accumulator
            timeAccumulator += robot.experiment_supervisor.getTime()
            
            # update the step number
            stepNumber += 1
            
            return 1
        
def moveForward(startingX, startingY, endingX, endingY, velo, distanceOffset, point1, point2, stepNum):
    
    global encoderReading
    global stepNumber
    global timeAccumulator
    
    
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
    printAnnounceValues(moveForward, velo, velo, estTime, distance, point1, point2)
    printNavValues(velo, velo, accumulatedDis-encoderOffset, robot.experiment_supervisor.getTime())
        
        
    # if the robot has traveled the distance, stop the robot
    if(accumulatedDis > distance+distanceOffset+encoderOffset):
        # Stop the robot and update the encoder readings only on the first time accumulatedDis > distance
        if accumulatedDis < distance+distanceOffset + 0.05 + encoderReading:
            
            # update the step number and encoder reading and stop the robot
            stepNumber += 1
            encoderReading = robot.get_front_right_motor_encoder_reading()
            
            # Grab the current time and add to the time accumulator
            timeAccumulator += robot.experiment_supervisor.getTime()
            
            # print new line to make output look nicer
            print('\n')
            
            robot.stop()
            
        return 1
        
def curvedTurn(radius, rads, direction, maxSpeed, distanceOffset, point1, point2, stepNum):
    global encoderReading
    global timeAccumulator
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
        
    # Calculate the angular velocity of the robot in order to determine the time it will take to reach the desired heading
    # Convert the angular velocity to linear velocity
    V_L = robot.wheel_radius * VeloLeft
    V_R = robot.wheel_radius * VeloRight
     
    # Calculate the angular velocity of the robot
    angularVelocity = (V_R - V_L)/robot.axel_length
    
    # Calculate the time it will take to reach the desired heading (2pi/|angular velocity|) also make sure angular velo is positivie
    estTime = rads / math.fabs(angularVelocity)
        
    # print out the announce values
    printAnnounceValues(curvedTurn, VeloLeft, VeloRight, estTime, distance, point1, point2)
    
    # print out the nav values
    printNavValues(VeloLeft, VeloRight, accumulatedDis-encoderOffset, robot.experiment_supervisor.getTime())

    
    # if the robot has traveled the distance, stop the robot
    if(accumulatedDis > rightDistance+encoderOffset + distanceOffset):
        # Stop the robot and update the encoder readings only on the first time accumulatedDis > distance
        if accumulatedDis < distance + encoderReading + distanceOffset:

            # update the step number and encoder reading and stop the robot
            stepNumber += 1
            encoderReading = robot.get_front_right_motor_encoder_reading()
            
            # Grab the current time and add to the time accumulator
            timeAccumulator += robot.experiment_supervisor.getTime()

            robot.stop()
        return 1
        
def printNavValues(VeloLeft, VeloRight, distance, time):
    global pollingCounter
    
    # Print out the values every 5 polling cycles
    if pollingCounter % 10 == 0:
        print('V_li = %0.1f, V_ri = %0.1f, D = %0.1f, T = %0.1f' % (VeloLeft, VeloRight, distance, time))
    pollingCounter += 1
    
def printAnnounceValues(func,VeloLeft, VeloRight, estimatedTime, distance, point1, point2, adjust= 0, heading = 0):
    global lastFuntion
    
    if adjust == 1:
        print('---------------------------------')
        print("Adjusting Heading to: %d" % heading)
        print("VeloLeft: ", VeloLeft)
        print("VeloRight: ", VeloRight)
        print('---------------------------------')
        print('\n')
        lastFuntion = func
        return
    
    if lastFuntion == func:
        return

    print('---------------------------------')
    print("Moving from: P%d, to P%d with velocities"  % (point1, point2))
    print("VeloLeft: ", VeloLeft)
    print("VeloRight: ", VeloRight)
    print("Estimated Time: %0.1f" % (estimatedTime))
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
    # Move from point P0 to P1 with velocity 20 rad/sec
    callFunction(moveForward, 2, -2, 2, -0.5, 20, -0.045, 0, 1, 0)
    
    # Ensure the robot is facing the correct direction before moving
    callFunction(rotate, 90, 2, 0, 1, 1, 1)
    
    # Move from point P1 to P2, left turn at 8 rad/sec
    callFunction(curvedTurn, 0.5, math.pi, 'left', 8, -0.045, 1, 2, 2)
    
    

    

    robot.experiment_supervisor.getTime()
    
    