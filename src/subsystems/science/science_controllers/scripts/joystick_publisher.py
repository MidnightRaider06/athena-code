#!/usr/bin/env python3
import os
import time
import pprint
import pygame
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

class JoystickPublisher(Node):

    def __init__(self):
   
        super().__init__('joystick')
        self.publisher_ = self.create_publisher(Joy, 'controller_input', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.controller_inputs)       


        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('joystick_type', rclpy.Parameter.Type.INTEGER),
            ]
        )

        # 0: PS4 1: Thrustmaster
        self.joystick_type = self.get_parameter('joystick_type').value
        self.get_logger().info("Joystick type: ")
        self.get_logger().info(str(self.joystick_type))


        joysticks = 0
        self.axis_data = None
        self.controller = None
        self.button_data = None
        self.hat_data = None
        self.activation = 0.08

        # Pygame Controller
        pygame.init()
        pygame.joystick.init()            
        joysticks = pygame.joystick.get_count()

        # Only begin once a joystick is connected
        while(joysticks == 0):
            # self.get_logger().info("No controllers are connected!")
            time.sleep(0.25)
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEADDED:
                    print("Joystick connected.")
                    pygame.joystick.init()            
                    joysticks = pygame.joystick.get_count()
                    break
            
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

        if not self.axis_data:
            self.axis_data = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0}

        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)

        self.previous_axes = np.zeros(6)
        self.previous_buttons = np.zeros(13)        

    def controller_inputs(self):
       
        joystick_vels = self.previous_axes
        button_activations = self.previous_buttons
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                self.axis_data[event.axis] = round(event.value,2)
            elif event.type == pygame.JOYBUTTONDOWN:
                self.button_data[event.button] = True
            elif event.type == pygame.JOYBUTTONUP:
                self.button_data[event.button] = False
            elif event.type == pygame.JOYHATMOTION:
                self.hat_data[event.hat] = event.value

            if(self.joystick_type == 0):    

                """
                Thrustmaster:
                Joystick Axes
                [0] = forward/backwards
                [1] = left/right
                [2] = slider
                [3] = empty
                [4] = empty
                [5] = empty

                Button Activations
                [0] = trigger
                [1] = bottom on stick
                [2] = left on stick
                [3] = right on stick
                [4] = left side - top left
                [5] = left side - top middle
                [6] = left side - top right
                [7] = left side - bottom right
                [8] = left side - bottom middle
                [9] = left side - bottom left
                [10] = right side - top right
                [11] = right side - top middle
                [12] = right side - top left
                """   
                self.get_logger().info(str(self.axis_data.get(2)))
                print(self.axis_data.get(2))

                # Up and Down
                if(self.axis_data.get(1) < -self.activation):
                    joystick_vels[0] = -((self.axis_data[1] + self.activation) / (1 -self.activation))  # normalizes the 0.25 to 1 range to a 0 - 1 range and then mult by max vel
                elif(self.axis_data.get(1) >self.activation):
                    joystick_vels[0] = -((self.axis_data[1] - self.activation) / (1 -self.activation)) 
                else:
                    joystick_vels[0] = 0

                # Left/Right
                if(self.axis_data.get(0) < -self.activation):
                    joystick_vels[1] = ((self.axis_data[3] + self.activation) / (1 -self.activation))  # normalizes the 0.25 to 1 range to a 0 - 1 range and then mult by max vel
                elif(self.axis_data.get(0) >self.activation):
                    joystick_vels[1] = ((self.axis_data[3] - self.activation) / (1 -self.activation)) 
                else:
                    joystick_vels[1] = 0

                # Slider
                if(self.axis_data.get(2) < -self.activation):
                    joystick_vels[2] = -((self.axis_data[4] + self.activation) / (1 -self.activation))  # normalizes the 0.25 to 1 range to a 0 - 1 range and then mult by max vel
                elif(self.axis_data.get(2) > self.activation):
                    joystick_vels[2] = -((self.axis_data[4] - self.activation) / (1 -self.activation)) 
                else:
                    joystick_vels[2] = 0
                
                # Buttons
                for i in range(13):
                    button_activations[i] = self.button_data[i]
            else:
                print('calibrate:', pprint.pprint(self.axis_data))

        # Save current numpy array for joystick and buttons
        self.previous_axes = joystick_vels
        self.previous_buttons = button_activations

        axes_data = joystick_vels.tolist()
        buttons_data = [int(el) for el in button_activations.tolist()]
        msg = Joy()
        msg.axes = axes_data
        msg.buttons = buttons_data


        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    joystick_publisher = JoystickPublisher()
    rclpy.spin(joystick_publisher)
    joystick_publisher.destroy_node()
    pygame.joystick.quit()
    pygame.quit()
    rclpy.shutdown()




if __name__ == '__main__':
    main()