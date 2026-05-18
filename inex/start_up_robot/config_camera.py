# ls /dev/ttyUSB*  # ตรวจสอบรายชื่อ ttyUSB ทั้งหมด
# ถ้ามองไม่เห็น /dev/ttyUSB0
# sudo apt remove brltty -y #ลบ Service ที่แย่งพอร์ตออก
# sudo modprobe ch341 #ถอดสาย USB ออกแล้วเสียบใหม่ หรือใช้คำสั่งบังคับให้ระบบโหลด Driver ใหม่
import serial
import time
def setup_wifi_module():
    port = '/dev/ttyUSB0' 
    baudrate = 115200
    
    # ANSI Escape Code สำหรับ Reset สีเป็นสีขาว/สีพื้นฐาน
    RESET = "\033[0m"
    
    print(f"--- ROS2 WiFi Configuration Tool ---{RESET}")
    print(f"Type 'q' at any time to exit the program.{RESET}")
    print(f"-------------------------------------{RESET}")

    while True:
        # รับค่าและล้างสีทันที
        user_input = input(f"\nEnter ROS2 Domain ID (or 'q' to quit): {RESET}").strip()

        if user_input.lower() == 'q':
            print(f"Exiting program. Goodbye!{RESET}")
            break

        if not user_input.isdigit():
            print(f">> Error: Please enter a valid number or 'q' to quit.{RESET}")
            continue

        try:
            ser = serial.Serial(port, baudrate, timeout=1)
            
            commands = [
                "wifi_ver",
                "wifi_mode:1",
                f"domain_id:{user_input}",
            ]
            
            for cmd in commands:
                full_cmd = cmd + "\r\n"
                ser.write(full_cmd.encode('utf-8'))
                print(f"Sending: {cmd}{RESET}")
                
                time.sleep(0.5)
                if ser.in_waiting > 0:
                    response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                    # ล้างสีหลังจากแสดง Response
                    print(f"Response: {response.strip()}{RESET}")
            
            ser.close()
            print(f">> Successfully set Domain ID to: {user_input}{RESET}")

        except Exception as e:
            print(f">> Serial Error: {e}{RESET}")
            print(f"Please check your connection and try again.{RESET}")

if __name__ == "__main__":
    setup_wifi_module()
