#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import threading
import tkinter as tk
from tkinter import ttk

class MotorGuiNode(Node):
    def __init__(self):
        super().__init__("motor_gui_node")
        self.pub1 = self.create_publisher(Int32, "/motor1/motor_cmd", 10)
        self.pub2 = self.create_publisher(Int32, "/motor2/motor_cmd", 10)

    def send_cmd(self, motor_id, value):
        msg = Int32()
        msg.data = value
        if motor_id == 1:
            self.pub1.publish(msg)
        else:
            self.pub2.publish(msg)


def ros_spin_thread(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main():
    # Inicializar ROS en thread aparte
    rclpy.init()
    node = MotorGuiNode()

    thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    thread.start()

    # ---- GUI en el HILO PRINCIPAL ----
    root = tk.Tk()
    root.title("Maki Motor Controller")

    # ===== MOTOR 1 =====
    tk.Label(root, text="Motor 1").pack()

    entry1 = tk.Entry(root)
    entry1.pack()

    slider1 = tk.Scale(root, from_=0, to=1023, orient="horizontal")
    slider1.pack()

    def send_m1():
        try:
            value = int(entry1.get()) if entry1.get() else slider1.get()
            node.send_cmd(1, value)
        except ValueError:
            print("Valor inválido para Motor 1")

    tk.Button(root, text="Enviar M1", command=send_m1).pack(pady=4)

    # ===== MOTOR 2 =====
    tk.Label(root, text="Motor 2").pack()

    entry2 = tk.Entry(root)
    entry2.pack()

    slider2 = tk.Scale(root, from_=0, to=1023, orient="horizontal")
    slider2.pack()

    def send_m2():
        try:
            value = int(entry2.get()) if entry2.get() else slider2.get()
            node.send_cmd(2, value)
        except ValueError:
            print("Valor inválido para Motor 2")

    tk.Button(root, text="Enviar M2", command=send_m2).pack(pady=4)

    # Ejecutar GUI
    root.mainloop()


if __name__ == "__main__":
    main()
