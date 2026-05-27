import json
import matplotlib.pyplot as plt

def load_data(filepath):
    """ฟังก์ชันสำหรับอ่านไฟล์และแปลงข้อมูลเป็น JSON โดยข้ามบรรทัดคอมเมนต์"""
    with open(filepath, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    # กรองบรรทัดที่ขึ้นต้นด้วย '#' ออก แล้วเชื่อมข้อความที่เหลือเข้าด้วยกัน
    json_str = "".join([line for line in lines if not line.strip().startswith('#')])
    return json.loads(json_str)

# 1. โหลดข้อมูลจากไฟล์ทั้ง 2 ไฟล์
pose_file = r"New_Audi_etron_x=16_yaw=35_pose.txt"
steer_file =r"New_Audi_etron_x=16_yaw=35_steer.txt"

pose_data = load_data(pose_file)
steer_data = load_data(steer_file)

# 2. แยกข้อมูลของ Pose Data 
# รูปแบบ: [x, y, elapsed_s, velocity_ms, accel_ms2, jerk_ms3, direction]
x_pos = [row[0] for row in pose_data]
y_pos = [row[1] for row in pose_data]
time_pose = [row[2] for row in pose_data]
velocity = [row[3] for row in pose_data]
acceleration = [row[4] for row in pose_data]
jerk = [row[5] for row in pose_data]

# 3. แยกข้อมูลของ Steer Data
# รูปแบบ: [elapsed_seconds, wheel_angle_degrees]
time_steer = [row[0] for row in steer_data]
steer_angle = [row[1] for row in steer_data]

# 4. สร้างหน้าต่างกราฟแบบ 2 แถว 3 คอลัมน์ (รวม 6 กราฟ)
fig, axs = plt.subplots(2, 3, figsize=(18, 10))
fig.tight_layout(pad=5.0)
max =75
# Graph 1: X Position vs Y Position (Trajectory)
axs[0, 0].plot(x_pos, y_pos, 'b-')
axs[0, 0].set_title('Graph 1: Trajectory (X vs Y)')
axs[0, 0].set_xlabel('X Position')
axs[0, 0].set_ylabel('Y Position')
axs[0, 0].set_xlim(0, 21) # กำหนดแกน x (X Position) ให้สุดที่ 25
axs[0, 0].grid(True)

# Graph 2: Time vs Velocity
axs[0, 1].plot(time_pose, velocity, 'g-')
axs[0, 1].set_title('Graph 2: Velocity over Time')
axs[0, 1].set_xlabel('Time (s)')
axs[0, 1].set_ylabel('Velocity (m/s)')
axs[0, 1].set_xlim(0, max) # กำหนดแกน x (Time) ให้สุดที่ 25
axs[0, 1].grid(True)

# Graph 3: Time vs X and Y Position
axs[0, 2].plot(time_pose, x_pos, 'r-', label='X Position')
axs[0, 2].plot(time_pose, y_pos, 'm-', label='Y Position')
axs[0, 2].set_title('Graph 3: X & Y Position over Time')
axs[0, 2].set_xlabel('Time (s)')
axs[0, 2].set_ylabel('Position')
axs[0, 2].set_xlim(0, max) # กำหนดแกน x (Time) ให้สุดที่ 25
axs[0, 2].legend()
axs[0, 2].grid(True)

# Graph 4: Time vs Acceleration
axs[1, 0].plot(time_pose, acceleration, 'c-')
axs[1, 0].set_title('Graph 4: Acceleration over Time')
axs[1, 0].set_xlabel('Time (s)')
axs[1, 0].set_ylabel('Acceleration (m/s²)')
axs[1, 0].set_xlim(0, max) # กำหนดแกน x (Time) ให้สุดที่ 25
axs[1, 0].grid(True)

# Graph 5: Time vs Jerk
axs[1, 1].plot(time_pose, jerk, 'y-')
axs[1, 1].set_title('Graph 5: Jerk over Time')
axs[1, 1].set_xlabel('Time (s)')
axs[1, 1].set_ylabel('Jerk (m/s³)')
axs[1, 1].set_xlim(0, max) # กำหนดแกน x (Time) ให้สุดที่ 25
axs[1, 1].grid(True)

# Graph 6: Time vs Steer Angle
axs[1, 2].plot(time_steer, steer_angle, 'k-')
axs[1, 2].set_title('Graph 6: Steer Angle over Time')
axs[1, 2].set_xlabel('Time (s)')
axs[1, 2].set_ylabel('Steer Angle (degrees)')
axs[1, 2].set_xlim(0, max) # กำหนดแกน x (Time) ให้สุดที่ 25
axs[1, 2].grid(True)

# แสดงกราฟทั้งหมด
plt.show()