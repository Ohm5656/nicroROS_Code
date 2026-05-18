#!/usr/bin/env python3
# encoding: utf-8

import sys, select, termios, tty, time, threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformListener, Buffer # เพิ่ม TF
import yaml # เพิ่มสำหรับ Save YAML

msg = """
Control Your Yahboom Robot & Servo!
----------------------------------
Moving around:
    u    i    o
    j    k    l
    m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

Special Actions:
    p : Run Servo Sequence (Step -40 then 0)
    s : SAVE Current Pose as Waypoint (to nav_waypoints.yaml)

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0), 'o': (1, -1), 'j': (0, 1), 'l': (0, -1),
    'u': (1, 1), ',': (-1, 0), '.': (-1, 1), 'm': (-1, -1),
}

speedBindings = {
    'q': (1.1, 1.1), 'z': (.9, .9),
    'w': (1.1, 1), 'x': (.9, 1),
    'e': (1, 1.1), 'c': (1, .9),
}

class Yahboom_Keyboard(Node):
    def __init__(self, name):
        super().__init__(name)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_servo = self.create_publisher(Int32, 'servo_s1', 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # เพิ่ม TF Listener สำหรับหาตำแหน่งหุ่นยนต์
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.declare_parameter("linear_speed_limit", 1.0)
        self.declare_parameter("angular_speed_limit", 5.0)
        self.linear_speed_limit = self.get_parameter("linear_speed_limit").get_parameter_value().double_value
        self.angular_speed_limit = self.get_parameter("angular_speed_limit").get_parameter_value().double_value
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.servo_busy = False
        self.waypoints = [] # เก็บ Waypoints
        
        self.dist_front = 10.0
        self.dist_back  = 10.0
        self.dist_left  = 10.0
        self.dist_right = 10.0
        self.safety_limit = 0.3
        self.last_beep_time = 0.0

    def scan_callback(self, msg):
        num_points = len(msg.ranges)
        b_idx = list(range(0, 31)) + list(range(num_points-30, num_points))
        r_idx = list(range(60, 121))
        f_idx = list(range(150, 211))
        l_idx = list(range(240, 301))

        def get_min_dist(indices):
            ranges = [msg.ranges[i] for i in indices if i < len(msg.ranges) and 0.05 < msg.ranges[i] < 3.0]
            return min(ranges) if ranges else 10.0

        self.dist_front = get_min_dist(f_idx)
        self.dist_left  = get_min_dist(l_idx)
        self.dist_back  = get_min_dist(b_idx)
        self.dist_right = get_min_dist(r_idx)

    def save_waypoint(self):
        """ ฟังก์ชันดึงตำแหน่งปัจจุบันจาก TF และบันทึกลง YAML """
        try:
            # ดึง Transform ล่าสุดจาก map ไป base_link
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now, 
                                                    timeout=rclpy.duration.Duration(seconds=0.2))
            
            pos = trans.transform.translation
            ori = trans.transform.rotation
            
            w_idx = len(self.waypoints) + 1
            wp_data = {
                'task': f'waypoint_{w_idx}',
                'x': round(pos.x, 3),
                'y': round(pos.y, 3),
                'orientation': {
                    'z': round(ori.z, 5),
                    'w': round(ori.w, 5)
                }
            }
            
            self.waypoints.append(wp_data)
            
            with open('nav_waypoints.yaml', 'w') as f:
                yaml.dump({'waypoints': self.waypoints}, f, sort_keys=False)
            
            print(f"\n[SAVED] {wp_data['task']} at x:{wp_data['x']}, y:{wp_data['y']}")
            
        except Exception as e:
            print(f"\n[ERROR] Could not save waypoint: {e}")

    def play_warning_sound(self):
        current_time = time.time()
        if current_time - self.last_beep_time > 0.5:
            sys.stdout.write('\a')
            sys.stdout.flush()
            self.last_beep_time = current_time
    
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_servo_sequence(self):
        self.servo_busy = True
        msg_servo = Int32(data=-40)
        self.pub_servo.publish(msg_servo)
        time.sleep(1.0)
        msg_servo.data = 0
        self.pub_servo.publish(msg_servo)
        self.servo_busy = False
        
    def vels(self, speed, turn):
        return f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}"

def main():
    rclpy.init()
    node = Yahboom_Keyboard("yahboom_keyboard_ctrl")
    
    (speed, turn) = (0.15, 0.8)
    (x, th) = (0, 0)
    status = 0
    count = 0

    try:
        print(msg)
        print(node.vels(speed, turn))
        
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = node.getKey()

            # --- ตรวจสอบปุ่มกดพิเศษ ---
            if key == 'p':
                if not node.servo_busy:
                    threading.Thread(target=node.run_servo_sequence).start()
            
            elif key == 's': # กด s เพื่อเซฟตำแหน่ง
                node.save_waypoint()

            # --- Logic การเคลื่อนที่ ---
            if key in moveBindings.keys():
                x, th = moveBindings[key]
                count = 0
            elif key in speedBindings.keys():
                speed = min(speed * speedBindings[key][0], node.linear_speed_limit)
                turn = min(turn * speedBindings[key][1], node.angular_speed_limit)
                print(node.vels(speed, turn))
                count = 0
            elif key == ' ' or key == 'k':
                x, th = 0, 0
            elif key == '\x03': # CTRL-C
                break
            else:
                count += 1
                if count > 4:
                    x, th = 0, 0

            # --- ระบบความปลอดภัย (Safety System) ---
            twist = Twist()
            target_linear = speed * x
            target_angular = turn * th
            is_blocked = False

            if x > 0 and node.dist_front < node.safety_limit:
                target_linear, is_blocked = 0.0, True
            elif x < 0 and node.dist_back < node.safety_limit:
                target_linear, is_blocked = 0.0, True

            if th > 0 and node.dist_left < node.safety_limit:
                target_linear, is_blocked = 0.0, True
            elif th < 0 and node.dist_right < node.safety_limit:
                target_linear, is_blocked = 0.0, True
                
            if is_blocked:
                node.play_warning_sound()
                
            twist.linear.x = float(target_linear)
            twist.angular.z = float(target_angular)
            node.pub.publish(twist)
            sys.stdout.flush()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.pub.publish(Twist())
        node.pub_servo.publish(Int32(data=0))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
