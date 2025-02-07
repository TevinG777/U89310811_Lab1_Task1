# Import MyRobot Class
from fairis_tools.my_robot import MyRobot
import math

stepCounter = 0
pollingCounter = 0
encoderReading = 0
totalRadtoRemove = 0
stepNumber = 0


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
        leftSpeed = -speed
        rightSpeed = speed
        robot.set_left_motors_velocity(leftSpeed)
        robot.set_right_motors_velocity(rightSpeed)
        
        print("Current Heading: ", tmpCurrentHeading)
        print("Desired Heading: ", tmpDesiredHeading)
        
        if(robot.get_compass_reading() >= desiredHeading):
            robot.stop()
            stepNumber += 1

            return 1
    else:
        leftSpeed = speed
        rightSpeed = -speed
        robot.set_left_motors_velocity(leftSpeed)
        robot.set_right_motors_velocity(rightSpeed)
        
        if(robot.get_compass_reading() <= desiredHeading):
            robot.stop()
            stepNumber += 1
            
            return 1
    
        
def moveForward(startingX, startingY, endingX, endingY, velo, distanceOffset, stepNum):
    
    global pollingCounter
    global encoderReading
    global stepNumber
    
    #
    
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
        
        # Set the motors to max speed (26 Rad/s)
        robot.set_right_motors_velocity(velo)
        robot.set_left_motors_velocity(velo)
        
        
        
        # Print out the robot metrics every 5 polling cycles
        if pollingCounter % 10 == 0:
            # print out robot metrics every multiple of 5 to reduce the amount of output
            print(f"V_li: {velo:.1f}, V_ri: {velo:.1f}, D_i: {accumulatedDis:.2f}, T_i: {robot.experiment_supervisor.getTime():.2f}")
        
    # if the robot has traveled the distance, stop the robot
    if(accumulatedDis > distance+distanceOffset+encoderOffset):
        # Stop the robot and update the encoder readings only on the first time accumulatedDis > distance
        if accumulatedDis < distance+distanceOffset + 0.05 + encoderReading:
            # stop the robot and exit
            
            print(f"V_li: {velo:.1f}, V_ri: {velo:.1f}, D_i: {accumulatedDis:.2f}, T_i: {robot.experiment_supervisor.getTime():.2f}")
            
            stepNumber += 1
            encoderReading = robot.get_front_right_motor_encoder_reading()
            print("Got here")
            robot.stop()
        return 1
        
    pollingCounter += 1
    
def curvedTurn(radius, rads, direction, maxSpeed, distanceOffset, stepNum):
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
    
    
    # Only print out the values if the funtion has not yet be completed
    if accumulatedDis < rightDistance + encoderOffset :
        if pollingCounter % 10 == 0:
            robot.set_left_motors_velocity(VeloLeft)
            robot.set_right_motors_velocity(VeloRight)
            print(f"V_li: {VeloLeft:.1f}, V_ri: {VeloRight:.1f}, D_i: {accumulatedDis:.2f}, T_i: {robot.experiment_supervisor.getTime():.2f}")
        
    
    
    
    
    # if the robot has traveled the distance, stop the robot
    if(accumulatedDis > rightDistance+encoderOffset + distanceOffset):
        # Stop the robot and update the encoder readings only on the first time accumulatedDis > distance
        if accumulatedDis < distance + encoderReading + distanceOffset:
            # stop the robot and exit
            print(f"V_li: {VeloLeft:.1f}, V_ri: {VeloRight:.1f}, D_i: {accumulatedDis:.2f}, T_i: {robot.experiment_supervisor.getTime():.2f}")
            stepNumber += 1
            encoderReading = robot.get_front_right_motor_encoder_reading()

            robot.stop()
        return 1
        
    pollingCounter += 1

# Main Control Loop for Robot
while robot.experiment_supervisor.step(robot.timestep) != -1:
    
    if(stepCounter == 0):
        # Print out V_ri and V_Li
        print("Moving from P0 to P1 with velocities:")
        print("V_ri = ", 20)
        print("V_Li = ", 20)
        stepCounter += 1
    
    # Pass starting x,y coords as well as a distance offset to account for momentum and set it as the 0st step
    nextCounter = moveForward(2.0, -2.0, 2.0, -0.5, 20, -0.05, 0)
    
    # After the robot has moved we need to proceed to P2
    if stepCounter == 1 and nextCounter == 1:
        # Print out V_ri and V_Li
        print("Moving from P1 to P2 with velocities:")
        print("V_ri =  Need to change")
        print("V_ri =  Need to change")
        stepCounter += 1
        nextCounter = 0
    
    # add offset to the left turn to account for momentum and set it as the 1st step
    nextCounter = curvedTurn(0.5, math.pi, "left", 8, -0.07,1)
    
    # after make sure the robot is facing 270 degrees
    if stepCounter == 2 and nextCounter == 1:
        # Print out V_ri and V_Li
        print("Rotating to 270 degrees with velocities:")
        print("V_ri =  Need to change")
        print("V_ri =  Need to change")
        stepCounter += 1
        nextCounter = 0
        
    # rotate the robot to 270 degrees
    nextCounter = rotate(270, 4, 2)
    
    # After the robot has moved we need to proceed to P3
    if stepCounter == 3 and nextCounter == 1:
        # Print out V_ri and V_Li
        print("Moving from P2 to P3 with Velocities:")
        print("V_ri =  Need to change")
        print("V_ri =  Need to change")
        stepCounter += 1
    # add offset to the right turn to account for momentum and set it as the 1st step
    nextCounter = curvedTurn(1.5, math.pi, "right", 10, -0.07,3)
    
    # after make sure the robot is facing 90 degrees
    if stepCounter == 4 and nextCounter == 1:
        # Print out V_ri and V_Li
        print("Rotating to 90 degrees with velocities:")
        print("V_ri =  Need to change")
        print("V_ri =  Need to change")
        stepCounter += 1
        nextCounter = 0
    nextCounter = rotate(90, 4, 4)
    
    if(stepCounter == 5 and nextCounter == 1):
        # Print out V_ri and V_Li
        print("Moving from P3 to P4 with velocities:")
        print("V_ri = ", 20)
        print("V_Li = ", 20)
        stepCounter += 1
        nextCounter = 0
    
    # Pass starting x,y coords as well as a distance offset to account for momentum and set it as the 0st step
    nextCounter = moveForward(-2.0, -0.5, -2.0, 2, 20, -0.05, 5)
    
    # Rotate the robot to 0 degrees
    if stepCounter == 6 and nextCounter == 1:
        # Print out V_ri and V_Li
        print("Rotating to 0 degrees with velocities from P4 to P5:")
        print("V_ri =  Need to change")
        print("V_ri =  Need to change")
        stepCounter += 1
        nextCounter = 0
    nextCounter = rotate(0, 4, 6)
    
    if(stepCounter == 7 and nextCounter == 1):
        # Print out V_ri and V_Li
        print("Moving from P3 to P4 with velocities:")
        print("V_ri = ", 20)
        print("V_Li = ", 20)
        stepCounter += 1
        nextCounter = 0
    
    # Pass starting x,y coords as well as a distance offset to account for momentum and set it as the 0st step
    nextCounter = moveForward(-2.0, 2, 1.5, 2, 20, -0.3, 7)
    
    

    robot.experiment_supervisor.getTime()
    
    