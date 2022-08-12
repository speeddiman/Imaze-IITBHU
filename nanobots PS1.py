"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor,DistanceSensor

def run_robot(robot):
    # create the Robot instance.
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28

    #Enable motors
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')

    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)

    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    #enable proximity sensor
    prox_sensors = []
    for ind in range(8):
        name = 'ps' + str(ind)
        prox_sensors.append(robot.getDevice(name))
        prox_sensors[ind].enable(timestep)

    # main loop
    # perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
     
    # Read the sensors
     sensors = [0, 2, 5, 7]
     for ind in sensors:
          print("ind: {}, val: {}".format(ind, prox_sensors[ind].getValue()))
         
         
     left_wall = prox_sensors[5].getValue()
     front_wall = prox_sensors[7].getValue() + prox_sensors[0].getValue()
     front_left = prox_sensors[7].getValue()
     front_right = prox_sensors[0].getValue()
     right_wall = prox_sensors[2].getValue()
     
     left_speed = max_speed
     right_speed = max_speed
     
     if front_wall > 155:
         if front_left >150 and front_right <68 :
            left_speed = max_speed
            right_speed = -max_speed
                
         if front_left <68 and front_right >150:
            left_speed = -max_speed 
            right_speed = max_speed  
       
         if front_left>90 and front_right >90 :
             left_speed = max_speed 
             right_speed = -max_speed
             
         if left_wall >90 and right_wall < 60:
             print('turning right')
             left_speed = max_speed
             right_speed = -max_speed
         if right_wall > 90 and left_wall <60:
             print('turning left')
             left_speed = -max_speed
             right_speed = max_speed
         if right_wall > 75 and left_wall > 75:
             print('Rotating')
             left_speed = max_speed
             right_speed = -max_speed       
     
     # if front_right :
         # print('turning right')
         # left_speed = max_speed
         # right_speed = -max_spe
     else:  
        
        if left_wall <80 and right_wall <80:
            print('Going straight')
            left_speed = max_speed
            right_speed = max_speed
         
        if left_wall >200 and right_wall <200 :
            print('adjusting myself')
            left_speed = max_speed*0.5
            right_speed = max_speed*0.2
          
        if right_wall >200 and left_wall <200 :
            print ('adjusting  myself')
            left_speed = max_speed*0.2
            sright_speed = max_speed*0.5
                    
        if front_left <75 and left_wall < 70:
            left_speed = -max_speed
            right_speed = max_speed
            
        if front_right <75  and left_wall< 70:
            left_speed = max_speed
            right_speed = max_speed      
      
     # else:
         # if left_wall:
         # #if left_wall :
            # print('going straight')
            # left_speed = max_speed
            # right_speed = m
         # else :
            # print ('Turn left')
            # left_speed = max_speed/3.5
            # right_speed = max_speed
         
     # Process sensor data here.
     left_motor.setVelocity(left_speed)
     right_motor.setVelocity(right_speed)
my_robot = Robot()
run_robot(my_robot)