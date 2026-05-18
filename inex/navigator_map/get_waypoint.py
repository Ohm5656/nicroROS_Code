import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import yaml
import sys, tty, termios
import select

class WaypointSaver(Node):
    def __init__(self):
        super().__init__('waypoint_saver')
        
        # ตั้งค่า TF Listener เพื่อดึงตำแหน่งหุ่นยนต์
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.waypoints = []
        self.filename = 'nav_waypoints.yaml'
        
        self.get_logger().info('--- Waypoint Saver (Robot Current Pose) ---')
        self.get_logger().info('1. Drive your robot to the desired location.')
        self.get_logger().info('2. Press "s" in THIS terminal to SAVE current robot pose.')
        self.get_logger().info('3. Press "Ctrl+C" to exit and finish.')

    def save_current_pose(self):
        try:
            # ดึงตำแหน่งล่าสุดจาก TF (จาก map ไปยังตัวหุ่นยนต์ base_link)
            now = rclpy.time.Time()
            # รอ transform 0.1 วินาทีเผื่อความชัวร์
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now, 
                                                    timeout=rclpy.duration.Duration(seconds=0.1))
            
            pos = trans.transform.translation
            ori = trans.transform.rotation
            
            current_count = len(self.waypoints) + 1
            waypoint_name = f'waypoint_{current_count}'
            
            waypoint_data = {
                'task': waypoint_name,
                'x': round(pos.x, 3),
                'y': round(pos.y, 3),
                'orientation': {
                    'z': round(ori.z, 5),
                    'w': round(ori.w, 5)
                }
            }
            
            self.waypoints.append(waypoint_data)
            
            # บันทึกลง YAML
            with open(self.filename, 'w') as f:
                yaml.dump({'waypoints': self.waypoints}, f, sort_keys=False)
            
            print(f"\n[SAVED] {waypoint_name}: x={pos.x:.3f}, y={pos.y:.3f}")
            
        except Exception as e:
            self.get_logger().warn(f'Could not get robot pose: {e}')

def get_key(settings):
    # ฟังก์ชันช่วยให้อ่านปุ่มกดได้โดยไม่ต้องกด Enter (Non-blocking check)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = WaypointSaver()
    settings = termios.tcgetattr(sys.stdin)

    try:
        while rclpy.ok():
            # รัน callback ของ ROS (เช่น TF update)
            rclpy.spin_once(node, timeout_sec=0.05)
            
            # ตรวจสอบการกดปุ่ม
            key = get_key(settings).lower()
            if key == 's':
                node.save_current_pose()
            elif key == '\x03': # Ctrl+C
                break

    except Exception as e:
        print(f"Error: {e}")
    finally:
        print(f'\nFinished! Total {len(node.waypoints)} waypoints saved in {node.filename}')
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
