"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor,DistanceSensor,Camera
import cv2 as cv
import numpy as np
import time
import math

def run_robot (robot) :
  # create the Robot instance.
  timestep = int(robot.getBasicTimeStep())
  max_speed = 6.27

  #Enable camera
  camera = robot.getDevice('camera')
  camera.enable(1)
    
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
     
       
  # Read the sensors
  f1=0
  f2=0
  f3=0
  f4=0
  f5=0
  f6=0   
  flag = 1 
  l=["green", "blue", "violet", "red"]   
   # main loop
  while robot.step(timestep) != -1:
      sensors = [0, 2, 5, 7]
      # for ind in sensors:
         # print("ind: {}, val: {}".format(ind, prox_sensors[ind].getValue()))
         
      left_wall = prox_sensors[5].getValue()
      front_wall = prox_sensors[7].getValue() + prox_sensors[0].getValue()
      front_left = prox_sensors[7].getValue()
      front_right = prox_sensors[0].getValue()
      right_wall = prox_sensors[2].getValue()
    
      left_speed = max_speed
      right_speed = -max_speed
  
      
      camera.saveImage('image.jpg' , 100)
      image = cv.imread('image.jpg')
      hsv_frame = cv.resize(image , (200,200))
      #cv.imshow('image_rgb' , image )
      
      left_motor.setVelocity(max_speed/2)
      right_motor.setVelocity(-max_speed/2)
      
      # h_min = 51 
      # s_min = 123
      # v_min = 0
      # h_max = 179
      # s_max = 255
      # v_max = 255
        
      # lower = np.array([h_min , s_min , v_min] , np.uint8)
      # upper = np.array([h_max , s_max , v_max] , np.uint8)
      # masked_image = cv.inRange(hsv_frame , lower , upper)
      
       
      
    
      #for blue colour
      
    
       
      #for green colour
      
       
      #for violet colour  
      
             
      if (flag == 1) :
          
          green_lower = np.array([0,50,0] , np.uint8)
          green_upper = np.array([55,255,55] , np.uint8)
          green_mask = cv.inRange(hsv_frame , green_lower , green_upper)         
          
          cv.imshow( 'real time masking' , green_mask)
          cv.waitKey(timestep)
          contours , _1 = cv.findContours(green_mask , cv.RETR_LIST , cv.CHAIN_APPROX_NONE )
          
          if(len(contours)>= 1):
              contours = sorted(contours, key =lambda x:cv.contourArea(x) , reverse=True )
              if cv.contourArea(contours[0])> 90 :
                  
                  M = cv.moments(contours[0])
                  if(M['m00'] != 0 ) :
                       cx = int(M['m10']/M['m00'])
                       cy = int(M['m01']/M['m00'])
                       #print(cx,cv.contourArea(contours[0]),len(contours))
                       
                       if cx >= 70 and cx <=130 :
                          print('going straight')
                          left_speed = max_speed
                          right_speed = max_speed
                          if front_wall > 150 :
                               flag = 2
                               print("reached", l[flag-1])
                       elif cx <70 :
                          print ('object on the left')
                          left_speed = max_speed*0.8
                          right_speed = max_speed
                               
                       elif cx >130 :        
                          print ('object on the right')
                          left_speed = max_speed
                          right_speed = max_speed*0.8
          
                             
             
                           
                 
                     
                      
      elif (flag == 2) :
          if f1==0:
              if prox_sensors[0].getValue()>80.0 or prox_sensors[7].getValue()>80.0  or  prox_sensors[1].getValue()>80.0:
                 left_speed = max_speed/2
                 right_speed = -max_speed/2
                 print("r")
        
              else:
                 if prox_sensors[5].getValue()>80.0 or prox_sensors[4].getValue()>80.0 or prox_sensors[6].getValue()>80.0:
                     left_speed = max_speed
                     right_speed = max_speed
                     print("f")
                 else:
                     left_speed = -max_speed/2
                     right_speed = max_speed/2
                     print("l")  
          blue_lower = np.array([70,0,0] , np.uint8)
          blue_upper = np.array([255,60,40] , np.uint8)
          blue_mask = cv.inRange(hsv_frame , blue_lower , blue_upper)
          cv.imshow( 'real time masking' , blue_mask)
          cv.waitKey(timestep)
          #det_colour = cv.bitwise_and( masked_image, masked_image , mask = blue_mask)
          contours , _1 = cv.findContours(blue_mask , cv.RETR_LIST , cv.CHAIN_APPROX_NONE )
          
          if(len(contours)>= 1):
              contours = sorted(contours, key =lambda x:cv.contourArea(x) , reverse=True )
              if (cv.contourArea(contours[0])> 90) :
                  M = cv.moments(contours[0])
                  if(M['m00'] != 0 ) :
                       cx = int(M['m10']/M['m00'])
                       cy = int(M['m01']/M['m00'])
                       #print(cx,cv.contourArea(contours[0]),len(contours))
                       if f2==0 and (prox_sensors[0].getValue()>80.0 or prox_sensors[7].getValue()>80.0 or prox_sensors[6].getValue()>80.0 or prox_sensors[1].getValue()>80.0):
                           f1=0
                       else:
                           f2=1
                           f1=1
                           if cx >= 70 and cx <=130 :
                              print('going straight')
                              left_speed = max_speed
                              right_speed = max_speed
                              if front_wall > 150 :
                                   
                                   print("reached", l[flag-1])
                                   flag = 3
                           elif cx <70 :
                              print ('object on the left')
                              left_speed = max_speed*0.8
                              right_speed = max_speed
                                   
                           elif cx >130 :        
                              print ('object on the right')
                              left_speed = max_speed
                              right_speed = max_speed*0.8
                  
                                 
                  
                     
      elif (flag == 3) :
          if f3==0:
              if prox_sensors[0].getValue()>80.0 or prox_sensors[7].getValue()>80.0  or  prox_sensors[1].getValue()>80.0:
                 left_speed = max_speed/2
                 right_speed = -max_speed/2
                 print("r")
        
              else:
                 if prox_sensors[5].getValue()>80.0 or prox_sensors[4].getValue()>80.0 or prox_sensors[6].getValue()>80.0:
                     left_speed = max_speed
                     right_speed = max_speed
                     print("f")
                 else:
                     left_speed = -max_speed/2
                     right_speed = max_speed/2
                     print("l")  
          violet_lower = np.array([60,15,40], np.uint8)
          violet_upper = np.array([160,45,150] , np.uint8)
          violet_mask = cv.inRange(hsv_frame , violet_lower , violet_upper)
          cv.imshow( 'real time masking' , violet_mask)
          cv.waitKey(timestep)
          #det_colour = cv.bitwise_and( masked_image, masked_image , mask = violet_mask)
          contours , _1 = cv.findContours(violet_mask , cv.RETR_LIST , cv.CHAIN_APPROX_NONE )
          
          if(len(contours)>= 1):
              contours = sorted(contours, key =lambda x:cv.contourArea(x) , reverse=True )
              if (cv.contourArea(contours[0])> 90) :
                  M = cv.moments(contours[0])
                  if(M['m00'] != 0 ) :
                       cx = int(M['m10']/M['m00'])
                       cy = int(M['m01']/M['m00'])
                       #print(cx,cv.contourArea(contours[0]),len(contours))
                       if f4==0 and (prox_sensors[0].getValue()>80.0 or prox_sensors[7].getValue()>80.0 or prox_sensors[6].getValue()>80.0 or prox_sensors[1].getValue()>80.0):
                           f3=0
                       else:
                           f4=1
                           f3=1
                           if cx >= 70 and cx <=130 :
                              print('going straight')
                              left_speed = max_speed
                              right_speed = max_speed
                              if front_wall > 150 :
                                   print("reached", l[flag-1])
                                   flag = 4
                           elif cx <70 :
                              print ('object on the left')
                              left_speed = max_speed*0.8
                              right_speed = max_speed
                                   
                           elif cx >130 :        
                              print ('object on the right')
                              left_speed = max_speed
                              right_speed = max_speed*0.8
              
             
                     
      elif (flag == 4) :
          if f5==0:
              if prox_sensors[0].getValue()>80.0 or prox_sensors[7].getValue()>80.0  or  prox_sensors[1].getValue()>80.0:
                 left_speed = max_speed/2
                 right_speed = -max_speed/2
                 print("r")
        
              else:
                 if prox_sensors[5].getValue()>80.0 or prox_sensors[4].getValue()>80.0 or prox_sensors[6].getValue()>80.0:
                     left_speed = max_speed
                     right_speed = max_speed
                     print("f")
                 else:
                     left_speed = -max_speed/2
                     right_speed = max_speed/2
                     print("l")  
          red_lower = np.array([0,0,70] , np.uint8)
          red_upper = np.array([60,60,255] , np.uint8)
          red_mask = cv.inRange(hsv_frame , red_lower , red_upper)
          cv.imshow( 'real time masking' , red_mask)
          cv.waitKey(timestep)
          #det_colour = cv.bitwise_and( masked_image, masked_image , mask = red_mask)
          # cv.imshow('real time masking' , red_mask)
          contours , _1 = cv.findContours(red_mask , cv.RETR_LIST , cv.CHAIN_APPROX_NONE )
          
          if(len(contours)>= 1):
              contours = sorted(contours, key =lambda x:cv.contourArea(x) , reverse=True )
              if (cv.contourArea(contours[0])> 90) :
                  M = cv.moments(contours[0])
                  if(M['m00'] != 0 ) :
                       cx = int(M['m10']/M['m00'])
                       cy = int(M['m01']/M['m00'])
                       #print(cx,cv.contourArea(contours[0]),len(contours))
                       if f6==0 and (prox_sensors[0].getValue()>80.0 or prox_sensors[7].getValue()>80.0 or prox_sensors[6].getValue()>80.0 or prox_sensors[1].getValue()>80.0) :
                           f5=0
                       else:
                           f6=1
                           f5=1
                           if cx >= 70 and cx <=130 :
                              print('going straight')
                              left_speed = max_speed
                              right_speed = max_speed
                              if front_wall > 150 :
                                   print("reached", l[flag-1])
                                   flag = 5
                           elif cx <70 :
                              print ('object on the left')
                              left_speed = max_speed*0.8
                              right_speed = max_speed
                                   
                           elif cx >130 :        
                              print ('object on the right')
                              left_speed = max_speed
                              right_speed = max_speed*0.8
              
             
                     
      elif flag == 5:
         print('all task completed')
         left_speed = 0.0
         right_speed = 0.0
                  
      cv.waitKey(timestep)                  
    
      left_motor.setVelocity(left_speed)
      right_motor.setVelocity(right_speed)
my_robot = Robot()
run_robot(my_robot)