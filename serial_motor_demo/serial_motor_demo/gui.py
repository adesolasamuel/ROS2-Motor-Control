import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time
from tkinter import *
import math
from serial_motor_demo_msgs.msg import MotorCommand



class MotorGui(Node):

    def __init__(self):
        super().__init__('motor_gui')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE

        self.publisher = self.create_publisher(MotorCommand, 'motor_command', qos_profile)

        self.tk = Tk()
        self.tk.title("Serial Motor GUI")
        root = Frame(self.tk)
        root.pack(fill=BOTH, expand=True)

        

        Label(root, text="Serial Motor GUI").pack()

        mode_frame = Frame(root)
        mode_frame.pack(fill=X)


        self.mode_lbl = Label(mode_frame, text="ZZZZ")
        self.mode_lbl.pack(side=LEFT)

        m1_frame = Frame(root)
        m1_frame.pack(fill=X)
        Label(m1_frame, text="Motor 1").pack(side=LEFT)
        self.m1 = Scale(m1_frame, from_=-255, to=255, orient=HORIZONTAL)
        self.m1.pack(side=LEFT, fill=X, expand=True)

        m2_frame = Frame(root)
        m2_frame.pack(fill=X)
        Label(m2_frame, text="Motor 2").pack(side=LEFT)
        self.m2 = Scale(m2_frame, from_=-255, to=255, resolution=1, orient=HORIZONTAL)
        self.m2.pack(side=LEFT, fill=X, expand=True)

        self.m2.config(to=10)

        motor_btns_frame = Frame(root)
        motor_btns_frame.pack()
        Button(motor_btns_frame, text='Send Once', command=self.send_motor_once).pack(side=LEFT)
        Button(motor_btns_frame, text='Send Cont.', command=self.show_values, state="disabled").pack(side=LEFT)
        Button(motor_btns_frame, text='Stop Send', command=self.show_values, state="disabled").pack(side=LEFT)
        Button(motor_btns_frame, text='Stop Mot', command=self.stop_motors).pack(side=LEFT)

        self.set_mode(True)


    def show_values(self):
        print (self.m1.get(), self.m2.get())

    def send_motor_once(self):
        msg = MotorCommand()
        msg.is_pwm = self.pwm_mode
        msg.mot_1_req_rad_sec = float(self.m1.get())
        msg.mot_2_req_rad_sec = float(self.m2.get())

        self.publisher.publish(msg)

    def stop_motors(self):
        msg = MotorCommand()
        msg.is_pwm = self.pwm_mode
        msg.mot_1_req_rad_sec = 0.0
        msg.mot_2_req_rad_sec = 0.0
        self.publisher.publish(msg)

    def set_mode(self, new_mode):
        self.pwm_mode = new_mode
        self.mode_lbl.config(text="Mode: PWM")

        self.update_scale_limits()

    def update_scale_limits(self):
        self.m1.config(from_=-255, to=255, resolution=1)
        self.m2.config(from_=-255, to=255, resolution=1)

    def update(self):
        self.tk.update()


def main(args=None):
    
    rclpy.init(args=args)

    motor_gui = MotorGui()

    rate = motor_gui.create_rate(20)    
    while rclpy.ok():
        rclpy.spin_once(motor_gui)
        motor_gui.update()


    motor_gui.destroy_node()
    rclpy.shutdown()


