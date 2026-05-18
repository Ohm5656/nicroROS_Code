import math

z = 0.01041
w = 0.99995

# คำนวณหาค่า Yaw
yaw_rad = 2 * math.atan2(z, w)
yaw_deg = math.degrees(yaw_rad)

print(f"Yaw (Rad): {yaw_rad:.5f}")
print(f"Yaw (Deg): {yaw_deg:.2f}")
