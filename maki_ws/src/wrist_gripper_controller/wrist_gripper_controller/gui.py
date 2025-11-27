import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class GUI(Node):
    def __init__(self):
        super().__init__("wrist_gripper_gui")

        self.pub_wrist = self.create_publisher(Float32, "wrist/position", 10)
        self.pub_gripper = self.create_publisher(Float32, "gripper/position", 10)
        self.pub_velocity = self.create_publisher(Float32, "motion/velocity", 10)

        # Tkinter window
        self.win = tk.Tk()
        self.win.title("Wrist + Gripper Control")

        # Wrist
        ttk.Label(self.win, text="Wrist Angle").pack()
        self.wrist_scale = tk.Scale(self.win, from_=-90, to=90,
                                    orient=tk.HORIZONTAL, command=self.update_wrist)
        self.wrist_scale.pack(fill="x")

        # Gripper
        ttk.Label(self.win, text="Gripper Angle").pack()
        self.gripper_scale = tk.Scale(self.win, from_=0, to=80,
                                      orient=tk.HORIZONTAL, command=self.update_gripper)
        self.gripper_scale.pack(fill="x")

        # Velocity
        ttk.Label(self.win, text="Velocity").pack()
        self.vel_scale = tk.Scale(self.win, from_=0.1, to=5.0, resolution=0.1,
                                  orient=tk.HORIZONTAL, command=self.update_velocity)
        self.vel_scale.pack(fill="x")

    def update_wrist(self, val):
        self.pub_wrist.publish(Float32(data=float(val)))

    def update_gripper(self, val):
        self.pub_gripper.publish(Float32(data=float(val)))

    def update_velocity(self, val):
        self.pub_velocity.publish(Float32(data=float(val)))

    def start(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            self.win.update_idletasks()
            self.win.update()


def main(args=None):
    rclpy.init(args=args)
    gui = GUI()
    gui.start()


if __name__ == "__main__":
    main()
