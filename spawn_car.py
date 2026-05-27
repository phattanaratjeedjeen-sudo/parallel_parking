import carla
import pygame
import numpy as np
import time
import math
import json
import datetime
import csv
import keyboard

# --- Data Storage ---
sensor_data = {
    'camera': None,
    'lidar':  None,
    'front_obs':   {'dist': float('inf'), 'time': 0},
    'rear_obs':    {'dist': float('inf'), 'time': 0},
    'side_fl_obs': {'dist': float('inf'), 'time': 0},
    'side_rl_obs': {'dist': float('inf'), 'time': 0},
    'side_fr_obs': {'dist': float('inf'), 'time': 0},
    'side_rr_obs': {'dist': float('inf'), 'time': 0},
    'imu': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'compass': 0.0}
}

# --- UI ---
WIDTH, HEIGHT = 1000, 600

# --- Sensor callbacks ---
def camera_callback(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))[:, :, :3]
    sensor_data['camera'] = pygame.surfarray.make_surface(array[:, :, ::-1].swapaxes(0, 1))

def lidar_callback(lidar_data):
    data   = np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(data, (int(data.shape[0] / 4), 4))
    lidar_img = np.zeros((200, 200, 3), dtype=np.uint8)
    scale    = 4.0
    x_coords = 100 + (points[:, 1] * scale)
    y_coords = 100 - (points[:, 0] * scale)
    p_map    = np.vstack((x_coords, y_coords)).T
    valid    = (p_map[:, 0] >= 0) & (p_map[:, 0] < 200) & \
               (p_map[:, 1] >= 0) & (p_map[:, 1] < 200)
    p_map    = p_map[valid].astype(np.int32)
    for p in p_map:
        lidar_img[p[1], p[0]] = (0, 255, 0)
    lidar_img[98:102, 98:102] = (255, 0, 0)
    sensor_data['lidar'] = pygame.surfarray.make_surface(lidar_img.swapaxes(0, 1))

def imu_callback(imu_data):
    sensor_data['imu'] = {
        'roll':    imu_data.transform.rotation.roll,
        'pitch':   imu_data.transform.rotation.pitch,
        'yaw':     imu_data.transform.rotation.yaw,
        'compass': math.degrees(imu_data.compass)
    }

def make_obs_callback(key):
    def callback(event):
        sensor_data[key] = {'dist': event.distance, 'time': time.time()}
    return callback

# --- Helpers ---
def norm_yaw(yaw):
    """Normalise yaw to (-180, 180]."""
    return (yaw + 180.0) % 360.0 - 180.0

def parked_ok(loc, yaw):
    """True when car3 is acceptably inside the parking slot."""
    return (abs(loc.x - 10.0) < 1.2 and
            abs(loc.y - 10.0) < 0.6 and
            abs(norm_yaw(yaw)) < 5.0)

# ===========================================================================
# Parking constants
# ===========================================================================
THROTTLE_FWD   = 0.2
THROTTLE_REV   = 0.32
THROTTLE_FINE  = 0.22
TARGET_X       = 10.0
TARGET_Y       = 10.0
PULL_UP_X      = 14.5
ARC1_YAW_DEG   = 45
ARC2_Y_LIMIT   = 10
ARC2_YAW_OK    = 2.0
YAW_DECEL_ZONE = 10.0    # degrees – start reducing throttle when |yaw| enters this zone

# --- Steering constants (all steer values in one place) ---
MAX_STEER_ANGLE        = 30.0   # degrees – physical wheel limit (default, all states except state 2)
MAX_STEER_ANGLE_STATE2 = 30 # degrees – physical wheel limit applied only during state 2
STEER_FULL_RIGHT  = -1.0   # normalised – full lock right  (state 2, state 5 fwd/wait_rev)
STEER_FULL_LEFT   =  1.0   # normalised – full lock left   (state 5 rev/wait_fwd)
STEER_STRAIGHT    =  0.0   # normalised – wheels centred
STEER_UNWIND_STEP =  0.05  # normalised units/frame – rate to unwind steer in state 3

# ===========================================================================
# Rendering helper
# ===========================================================================
def render_frame(display, font, car3, ctrl,
                 park_state, retry_count, park_done,
                 view_mode, gear_reverse, mode_label):

    # Camera / background
    if view_mode == "external":
        display.fill((15, 15, 20))
        if sensor_data['camera']:
            display.blit(sensor_data['camera'], (0, 0))
        if sensor_data['lidar']:
            display.blit(sensor_data['lidar'], (WIDTH - 220, 20))
    else:
        display.fill((0, 0, 0))
        if sensor_data['camera']:
            display.blit(sensor_data['camera'], (0, 0))

        # Steering wheel
        wc = (WIDTH // 2, HEIGHT // 2 + 50)
        wr = 120
        sa = ctrl.steer * 90
        pygame.draw.circle(display, (150, 150, 150), wc, wr, 8)
        for i in range(4):
            ang   = math.radians(i * 90 + sa)
            start = (wc[0] + (wr - 20) * math.cos(ang), wc[1] + (wr - 20) * math.sin(ang))
            end   = (wc[0] + wr        * math.cos(ang), wc[1] + wr        * math.sin(ang))
            pygame.draw.line(display, (100, 100, 100), start, end, 6)

        for rect_x, color, val in [
            (WIDTH - 150, (0, 255, 0),  ctrl.throttle),
            (WIDTH - 100, (255, 0,   0), ctrl.brake),
        ]:
            r = pygame.Rect(rect_x, HEIGHT - 150, 40, 100)
            if val > 0:
                r.height = int(100 - val * 60)
                r.y     += 60 - r.height
            pygame.draw.rect(display, color, r)
            pygame.draw.rect(display, (255, 255, 255), r, 2)

        g_txt = font.render(f"Gear: {'R' if gear_reverse else 'D'}", True, (255, 255, 0))
        display.blit(g_txt, (WIDTH - 150, HEIGHT - 200))

    # ---- HUD overlay ----
    curr    = time.time()
    loc_now = car3.get_location()
    yaw_now = norm_yaw(car3.get_transform().rotation.yaw)

    def get_obs(k, label):
        v      = sensor_data[k]
        active = curr - v['time'] < 0.2
        color  = (255, 50, 50) if active else (50, 255, 50)
        txt    = f"{label}: {v['dist']:.2f}m" if active else f"{label}: Clear"
        return font.render(txt, True, color)

    # Park-state label
    if park_done:
        state_txt = "PARKED"
        state_col = (0, 255, 100)
    elif park_state > 0:
        labels = {1: "FWD pull-up",    2: "Arc1 rev-right",
                  3: "Stay still",     4: "Rev straight",
                  5: "Shunt loop",     6: "Parked",
                  7: f"Retry #{retry_count}"}
        state_txt = f"[{park_state}] {labels.get(park_state, '')}"
        state_col = (255, 100, 255)
    else:
        state_txt = "Initializing..."
        state_col = (180, 180, 180)

    # Wheel angle
    MAX_WHEEL_DEG = MAX_STEER_ANGLE
    wheel_deg     = ctrl.steer * MAX_WHEEL_DEG
    bar_filled = int(abs(ctrl.steer) * 40)
    bar_left   = "█" * (bar_filled if ctrl.steer < 0 else 0)
    bar_right  = "█" * (bar_filled if ctrl.steer > 0 else 0)
    wheel_bar  = f"{bar_left:>5}|{bar_right:<5}"
    wheel_col  = (255, 140, 0) if abs(ctrl.steer) > 0.05 else (120, 120, 120)

    ui_rows = [
        get_obs('front_obs',   "FRONT"),
        get_obs('rear_obs',    "REAR"),
        get_obs('side_fl_obs', "L-FRONT"),
        get_obs('side_rl_obs', "L-REAR"),
        get_obs('side_fr_obs', "R-FRONT"),
        get_obs('side_rr_obs', "R-REAR"),
        font.render("------------------",                                  True, (60, 60, 60)),
        font.render(f"  X  : {loc_now.x:>8.3f} m",                       True, (255, 230, 60)),
        font.render(f"  Y  : {loc_now.y:>8.3f} m",                       True, (255, 230, 60)),
        font.render(f"  Yaw: {yaw_now:>8.3f} deg",                       True, (255, 230, 60)),
        font.render(f"  Wheel:{wheel_deg:>+7.1f} deg",                   True, wheel_col),
        font.render(f"  L{wheel_bar}R",                                  True, wheel_col),
        font.render("------------------",                                  True, (60, 60, 60)),
        font.render(f"Gear : {'REVERSE' if ctrl.reverse else 'FORWARD'}", True, (0, 255, 200)),
        font.render(f"View : {view_mode.capitalize()}",                  True, (0, 255, 200)),
        font.render(f"Mode : {mode_label}",                              True, (100, 200, 255)),
        font.render(f"Tries: {retry_count}",                              True, (255, 180, 0)),
        font.render(state_txt,                                             True, state_col),
    ]

    bg = pygame.Surface((275, len(ui_rows) * 26 + 20))
    bg.set_alpha(210)
    bg.fill((0, 0, 0))
    display.blit(bg, (10, 10))
    for i, row in enumerate(ui_rows):
        display.blit(row, (18, 18 + i * 26))

    pygame.display.flip()

def calculate_dynamic_turning_radius(vehicle):
    physics = vehicle.get_physics_control()
    realistic_angle = MAX_STEER_ANGLE
    wheels = physics.wheels
    wheels[0].max_steer_angle = realistic_angle
    wheels[1].max_steer_angle = realistic_angle
    physics.wheels = wheels
    max_steer_deg = wheels[0].max_steer_angle
    max_steer_rad = math.radians(max_steer_deg)
    
    fl_wheel = physics.wheels[0].position  
    rl_wheel = physics.wheels[2].position  
    
    wheelbase = math.sqrt((fl_wheel.x - rl_wheel.x)**2 + (fl_wheel.y - rl_wheel.y)**2) / 100.0
    R_min = wheelbase / math.tan(max_steer_rad)
    R_actual = R_min * 1.05 
    
    print(f"--- Vehicle Specs Extracted ---")
    print(f"Wheelbase:   {wheelbase:.2f} m")
    print(f"Max Steer:   {max_steer_deg:.2f} deg")
    print(f"Theory R:    {R_min:.2f} m")
    print(f"Practical R: {R_actual:.2f} m")
    print(f"-------------------------------")
    
    return R_actual

# ===========================================================================
# Get Reference Ideal Path
# ===========================================================================
def get_reference_path(car, car1, car2):
    ideal_points = []
    R = calculate_dynamic_turning_radius(car)
    print(f"Using R = {R:.2f} m for reference path calculation.")

    # --- Derive all path dimensions from actual car sizes ---
    car3_start_y  = car.get_location().y
    slot_front_y  = car1.get_location().y
    slot_start_x  = car1.get_location().x + car1.bounding_box.extent.x
    slot_end_x    = car2.get_location().x - car2.bounding_box.extent.x
    diag_max_dist = car.bounding_box.extent.x * 2

    print(f"Path params – start_y:{car3_start_y:.2f}  slot_y:{slot_front_y:.2f}  ")
    print(f"             slot_x:[{slot_start_x:.2f} → {slot_end_x:.2f}]  diag_range:{diag_max_dist:.2f}")

    # 1. เส้นตรงเดินหน้า
    for x in np.arange(0.0, PULL_UP_X, 0.1):
        ideal_points.append(carla.Location(x=x, y=car3_start_y, z=0.2))

    # 2. โค้งถอยเข้าซอง (Arc 1)
    center_y = car3_start_y
    for angle in np.arange(90, 135, 1):
        rad = math.radians(angle)
        ix = PULL_UP_X - 1.3 + R * math.cos(rad)
        iy = center_y + R * math.sin(rad) - R
        ideal_points.append(carla.Location(x=ix, y=iy, z=0.2))

    # 3. ถอยทะแยง 45 องศา
    end_arc_x = PULL_UP_X + R * math.cos(math.radians(135)) - 1.3
    end_arc_y = center_y + R * math.sin(math.radians(135)) - R

    for step in np.arange(0.0, diag_max_dist, 0.05):
        ix = end_arc_x - step
        iy = end_arc_y - step
        if iy >= slot_front_y:
            ideal_points.append(carla.Location(x=ix, y=iy, z=0.2))

    # 4. เดินหน้าตั้งลำกลางช่องจอด
    for x in np.arange(slot_start_x, slot_end_x + 0.1, 0.1):
        ideal_points.append(carla.Location(x=x, y=slot_front_y, z=0.2))

    return ideal_points

# ===========================================================================
# State-5 yaw alignment check
# ===========================================================================
def check_yaw_aligned(car3, yaw, ctrl):
    """
    Always called at the top of State 5 every frame.
    - If |yaw| > 1.0  → not aligned yet; returns (False, False).
    - If |yaw| <= 1.0 → applies full stop controls and returns:
        (True, True)  when the car has fully stopped  (→ transition to State 6)
        (True, False) while the car is still rolling   (→ hold brakes, skip phase logic)
    """
    if abs(yaw) > 2.0:
        return False, False          # still needs shunting

    # Yaw is within ±1° – override all phase controls with a hard stop
    ctrl.throttle   = 0.0
    ctrl.brake      = 1.0
    ctrl.steer      = 0.0
    ctrl.reverse    = False
    ctrl.hand_brake = True

    vel   = car3.get_velocity()
    speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
    return True, speed < 0.02       # (aligned, fully_stopped)

# ===========================================================================
# Steering-physics helper
# ===========================================================================
def set_max_steer(vehicle, angle_deg):
    """Apply a new max_steer_angle (degrees) to the front wheels via physics control."""
    phys = vehicle.get_physics_control()
    wheels = phys.wheels
    wheels[0].max_steer_angle = angle_deg
    wheels[1].max_steer_angle = angle_deg
    phys.wheels = wheels
    vehicle.apply_physics_control(phys)
    print(f"[STEER] max_steer_angle set to {angle_deg}°")

# ===========================================================================
# MAIN
# ===========================================================================
def main():
    actor_list = []
    pygame.init()
    font    = pygame.font.SysFont("consolas", 17, bold=True)
    display = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("CARLA – Parallel Auto-Parking (Fully Autonomous)")

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        bp_lib = world.get_blueprint_library()
        veh_bp = bp_lib.filter('vehicle.audi.etron')[0]
        # veh_bp = bp_lib.filter('vehicle.tesla.model3')[0]
        # Parked cars (static)
        car1 = world.spawn_actor(veh_bp, carla.Transform(carla.Location(x=5,  y=9.5, z=3)))
        actor_list.append(car1)
        car2 = world.spawn_actor(veh_bp, carla.Transform(carla.Location(x=PULL_UP_X, y=9.5, z=3)))
        actor_list.append(car2)

        # Ego car
        car3_tf = carla.Transform(carla.Location(x=10, y=12, z=3))
        car3    = world.spawn_actor(veh_bp, car3_tf)
        actor_list.append(car3)

        # Top-down spectator
        world.get_spectator().set_transform(carla.Transform(
            car3_tf.location + carla.Location(z=30),
            carla.Rotation(pitch=-90)
        ))

        # --- Sensors ---
        cam = world.spawn_actor(bp_lib.find('sensor.camera.rgb'),
                                carla.Transform(carla.Location(x=1.5, z=2.4)),
                                attach_to=car3)
        cam.listen(camera_callback)
        actor_list.append(cam)

        lidar = world.spawn_actor(bp_lib.find('sensor.lidar.ray_cast'),
                                  carla.Transform(carla.Location(z=2.4)),
                                  attach_to=car3)
        lidar.listen(lidar_callback)
        actor_list.append(lidar)

        imu = world.spawn_actor(bp_lib.find('sensor.other.imu'),
                                carla.Transform(carla.Location(z=0.5)),
                                attach_to=car3)
        imu.listen(imu_callback)
        actor_list.append(imu)

        obs_bp = bp_lib.find('sensor.other.obstacle')
        obs_bp.set_attribute('distance', '8')
        for loc, rot, key in [
            [carla.Location(x=2.5,        z=0.5), carla.Rotation(),        'front_obs'],
            [carla.Location(x=-2.5,       z=0.5), carla.Rotation(yaw=180), 'rear_obs'],
            [carla.Location(x=1.2,  y=-1, z=0.5), carla.Rotation(yaw=-90), 'side_fl_obs'],
            [carla.Location(x=-1.2, y=-1, z=0.5), carla.Rotation(yaw=-90), 'side_rl_obs'],
            [carla.Location(x=1.2,  y=1,  z=0.5), carla.Rotation(yaw=90),  'side_fr_obs'],
            [carla.Location(x=-1.2, y=1,  z=0.5), carla.Rotation(yaw=90),  'side_rr_obs'],
        ]:
            s = world.spawn_actor(obs_bp, carla.Transform(loc, rot), attach_to=car3)
            s.listen(make_obs_callback(key))
            actor_list.append(s)

        # -----------------------------------------------------------------
        # Physics warm-up
        # -----------------------------------------------------------------
        for _ in range(20):
            world.tick() if hasattr(world, 'tick') else time.sleep(0.05)

        # =================================================================
        # แก้ไขระบบฟิสิกส์ให้สมจริง
        # =================================================================
        physics_control = car3.get_physics_control()
        realistic_steer_angle = MAX_STEER_ANGLE
        
        _wheels = physics_control.wheels
        _wheels[0].max_steer_angle = realistic_steer_angle
        _wheels[1].max_steer_angle = realistic_steer_angle
        physics_control.wheels = _wheels
        
        car3.apply_physics_control(physics_control)
        print(f"Applied Realistic Max Steer Angle: {realistic_steer_angle} degrees")
        
        world.tick() if hasattr(world, 'tick') else time.sleep(0.1)
        # =================================================================

        # --- Auto Parking Variables Initialization ---
        auto_park_state = 1
        retry_count     = 0
        state3_timer    = None 
        s5_phase        = 'rev' 
        s5_timer        = 0.0
        s5_action_count = 0 
        park_done       = False
        green_path_log  = []
        steer_log       = []  # สำหรับเก็บ Log ของมุมล้อจริง
        _run_start_time = time.time()
        _prev_speed     = 0.0
        _prev_accel     = 0.0
        _prev_log_time  = time.time()
        
        # CSV logging
        _csv_log    = []
        _csv_frame  = 0
        _csv_ts     = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        
        current_steer   = 0.0
        view_mode       = "external"
        t_pressed       = False
        clock           = pygame.time.Clock()

        # ตัวแปรสำหรับ PID
        pid_integral = 0.0
        pid_prev_error = 0.0
        last_time = time.time()

        # =========================================================
        # ดึงระยะ Offset ของเพลาล้อหลังแบบไดนามิก
        # =========================================================
        physics = car3.get_physics_control()
        rear_axle_offset = abs(physics.wheels[2].position.x) / 100.0

        # =========================================================
        # Print all vehicle model data
        # =========================================================
        bb   = car3.bounding_box.extent
        wh   = physics.wheels
        fl   = wh[0].position
        fr   = wh[1].position
        rl   = wh[2].position
        rr   = wh[3].position
        wheelbase  = abs(fl.x - rl.x) / 100.0
        track_width = abs(fl.y - fr.y) / 100.0

        print("="*55)
        print(f"  Vehicle : {car3.type_id}")
        print("-"*55)
        print(f"  Mass              : {physics.mass:.1f} kg")
        print(f"  Drag coefficient  : {physics.drag_coefficient:.4f}")
        print(f"  Max RPM           : {physics.max_rpm:.0f} rpm")
        print(f"  Moi               : {physics.moi:.4f}")
        print("-"*55)
        print(f"  Bounding box (L x W x H)")
        print(f"    Length : {bb.x*2:.3f} m")
        print(f"    Width  : {bb.y*2:.3f} m")
        print(f"    Height : {bb.z*2:.3f} m")
        print("-"*55)
        print(f"  Wheelbase         : {wheelbase:.3f} m")
        print(f"  Track width       : {track_width:.3f} m")
        print(f"  Rear axle offset  : {rear_axle_offset:.3f} m")
        print("-"*55)
        wheel_names = ["FL", "FR", "RL", "RR"]
        for i, w in enumerate(wh):
            print(f"  Wheel {wheel_names[i]}:")
            print(f"    Radius          : {w.tire_friction:.4f} (friction)")
            print(f"    Radius          : {w.radius:.2f} cm")
            print(f"    Max steer angle : {w.max_steer_angle:.2f} deg")
            print(f"    Damping rate    : {w.damping_rate:.4f}")
            print(f"    Max brake torque: {w.max_brake_torque:.2f} Nm")
            print(f"    Max handbrake   : {w.max_handbrake_torque:.2f} Nm")
            print(f"    Position (cm)   : x={w.position.x:.1f}, y={w.position.y:.1f}, z={w.position.z:.1f}")
        print("-"*55)
        for i, g in enumerate(physics.forward_gears):
            print(f"  Gear {i+1}: ratio={g.ratio:.3f}  up={g.up_ratio:.3f}  down={g.down_ratio:.3f}")
        print("="*55)
        print(f"Dynamic Rear Axle Offset: {rear_axle_offset:.3f} meters")

        # =========================================================
        # Export vehicle model + parking constants to CSV
        # =========================================================
        _model_fname = f"parking_model_{_csv_ts}.csv"
        with open(_model_fname, 'w', newline='') as _f:
            _w = csv.writer(_f)
            _w.writerow(['category', 'key', 'value', 'unit'])
            _w.writerow(['vehicle', 'type_id',           car3.type_id,             ''])
            _w.writerow(['vehicle', 'mass',              f"{physics.mass:.1f}",     'kg'])
            _w.writerow(['vehicle', 'drag_coefficient',  f"{physics.drag_coefficient:.4f}", ''])
            _w.writerow(['vehicle', 'max_rpm',           f"{physics.max_rpm:.0f}",  'rpm'])
            _w.writerow(['vehicle', 'moi',               f"{physics.moi:.4f}",      ''])
            _w.writerow(['dimensions', 'length',         f"{bb.x*2:.4f}",           'm'])
            _w.writerow(['dimensions', 'width',          f"{bb.y*2:.4f}",           'm'])
            _w.writerow(['dimensions', 'height',         f"{bb.z*2:.4f}",           'm'])
            _w.writerow(['dimensions', 'wheelbase',      f"{wheelbase:.4f}",        'm'])
            _w.writerow(['dimensions', 'track_width',    f"{track_width:.4f}",      'm'])
            _w.writerow(['dimensions', 'rear_axle_offset', f"{rear_axle_offset:.4f}", 'm'])
            for _i, _wn in enumerate(['FL','FR','RL','RR']):
                _ww = wh[_i]
                _w.writerow([f'wheel_{_wn}', 'radius',           f"{_ww.radius:.2f}",           'cm'])
                _w.writerow([f'wheel_{_wn}', 'tire_friction',     f"{_ww.tire_friction:.4f}",    ''])
                _w.writerow([f'wheel_{_wn}', 'max_steer_angle',   f"{_ww.max_steer_angle:.2f}",  'deg'])
                _w.writerow([f'wheel_{_wn}', 'damping_rate',      f"{_ww.damping_rate:.4f}",     ''])
                _w.writerow([f'wheel_{_wn}', 'max_brake_torque',  f"{_ww.max_brake_torque:.2f}", 'Nm'])
                _w.writerow([f'wheel_{_wn}', 'max_handbrake_torque', f"{_ww.max_handbrake_torque:.2f}", 'Nm'])
                _w.writerow([f'wheel_{_wn}', 'position_x',        f"{_ww.position.x:.2f}",       'cm'])
                _w.writerow([f'wheel_{_wn}', 'position_y',        f"{_ww.position.y:.2f}",       'cm'])
                _w.writerow([f'wheel_{_wn}', 'position_z',        f"{_ww.position.z:.2f}",       'cm'])
            for _i, _g in enumerate(physics.forward_gears):
                _w.writerow([f'gear_{_i+1}', 'ratio',    f"{_g.ratio:.4f}",    ''])
                _w.writerow([f'gear_{_i+1}', 'up_ratio', f"{_g.up_ratio:.4f}", ''])
                _w.writerow([f'gear_{_i+1}', 'down_ratio', f"{_g.down_ratio:.4f}", ''])
            _w.writerow(['steering', 'MAX_STEER_ANGLE',   MAX_STEER_ANGLE,   'deg'])
            _w.writerow(['steering', 'STEER_FULL_RIGHT',  STEER_FULL_RIGHT,  'normalised'])
            _w.writerow(['steering', 'STEER_FULL_LEFT',   STEER_FULL_LEFT,   'normalised'])
            _w.writerow(['steering', 'STEER_STRAIGHT',    STEER_STRAIGHT,    'normalised'])
            _w.writerow(['steering', 'STEER_UNWIND_STEP', STEER_UNWIND_STEP, 'normalised'])
            _w.writerow(['throttle', 'THROTTLE_FWD',  THROTTLE_FWD,  'normalised'])
            _w.writerow(['throttle', 'THROTTLE_REV',  THROTTLE_REV,  'normalised'])
            _w.writerow(['throttle', 'THROTTLE_FINE', THROTTLE_FINE, 'normalised'])
            _w.writerow(['state_condition', 'PULL_UP_X',    PULL_UP_X,    'm'])
            _w.writerow(['state_condition', 'ARC1_YAW_DEG', ARC1_YAW_DEG, 'deg'])
            _w.writerow(['state_condition', 'ARC2_Y_LIMIT', ARC2_Y_LIMIT, 'm'])
            _w.writerow(['state_condition', 'ARC2_YAW_OK',  ARC2_YAW_OK,  'deg'])
            _w.writerow(['pid', 'Kp', 0.7, ''])
            _w.writerow(['pid', 'Ki', 0.0, ''])
            _w.writerow(['pid', 'Kd', 0.7, ''])
            _w.writerow(['spawn', 'car1_x', car1.get_location().x, 'm'])
            _w.writerow(['spawn', 'car1_y', car1.get_location().y, 'm'])
            _w.writerow(['spawn', 'car2_x', car2.get_location().x, 'm'])
            _w.writerow(['spawn', 'car2_y', car2.get_location().y, 'm'])
            _w.writerow(['spawn', 'car3_start_x', car3.get_location().x, 'm'])
            _w.writerow(['spawn', 'car3_start_y', car3.get_location().y, 'm'])
        print(f"[CSV] Model data saved → {_model_fname}")

        # =========================================================
        # สร้างเส้น Reference Path และตัวแปรควบคุมเส้น
        # =========================================================
        ref_path = get_reference_path(car3, car1, car2)
        show_path = True
        c_pressed = False

        running = True
        # ===========================================================================
        # MAIN LOOP
        # ===========================================================================
        while running:
            clock.tick(60)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Toggle view (T)
            if keyboard.is_pressed('t'):
                if not t_pressed:
                    view_mode = "cockpit" if view_mode == "external" else "external"
                    t_pressed = True
            else:
                t_pressed = False

            # ตรวจสอบปุ่ม 'C'
            if keyboard.is_pressed('c'):
                if not c_pressed:
                    show_path = not show_path  
                    c_pressed = True           
            else:
                c_pressed = False              

            # =========================================================
            # พล็อตพิกัดรถและหา Cross-Track Error (CTE)
            # =========================================================
            loc = car3.get_location()
            yaw = norm_yaw(car3.get_transform().rotation.yaw)
            yaw_rad = math.radians(yaw)

            rear_x = loc.x - (rear_axle_offset * math.cos(yaw_rad))
            rear_y = loc.y - (rear_axle_offset * math.sin(yaw_rad))

            car_plot_loc = carla.Location(x=loc.x, y=loc.y, z=0.2)

            min_dist = float('inf')
            closest_point = None
            for p in ref_path:
                dist = math.sqrt((car_plot_loc.x - p.x)**2 + (car_plot_loc.y - p.y)**2)
                if dist < min_dist:
                    min_dist = dist
                    closest_point = p

            # --- [FIXED] ดึงเวกเตอร์ความเร็ว และความเร่งแท้จริงตรงจาก Physics Engine ของ CARLA ---
            _vel_vec       = car3.get_velocity()
            _accel_vec     = car3.get_acceleration()
            _forward_vec   = car3.get_transform().get_forward_vector()
            
            # 1. คำนวณความเร็ว (สเกลาร์)
            _cur_speed     = math.sqrt(_vel_vec.x**2 + _vel_vec.y**2 + _vel_vec.z**2)
            
            # 2. หาความเร่งเชิงเส้นตามแนวแกนรถ (ขจัดสัญญาณรบกวนจากการดิฟเวลาทิ้ง)
            _cur_accel     = (_accel_vec.x * _forward_vec.x + 
                              _accel_vec.y * _forward_vec.y + 
                              _accel_vec.z * _forward_vec.z)
            
            # 3. ดึง Sim Delta Time จากโลกจำลองเพื่อให้การหาค่า Jerk มีความสมดุล
            _snapshot      = world.get_snapshot()
            _dt_log        = _snapshot.timestamp.delta_seconds if _snapshot else (time.time() - _prev_log_time)
            if _dt_log <= 0: _dt_log = 1.0 / 60.0
            
            _cur_jerk      = (_cur_accel - _prev_accel) / _dt_log
            _cur_time      = time.time()
            _elapsed       = round(_cur_time - _run_start_time, 3)

            # --- [ADDED] คำนวณทิศทางการเคลื่อนที่ของรถจริงด้วยสมบัติ Dot Product ---
            _motion_dot    = _vel_vec.x * _forward_vec.x + _vel_vec.y * _forward_vec.y + _vel_vec.z * _forward_vec.z
            if _motion_dot > 0.02:
                _direction = "FORWARD"
            elif _motion_dot < -0.02:
                _direction = "REVERSE"
            else:
                _direction = "STATIONARY"

            if show_path:
                # for i in range(len(ref_path) - 1):
                #     world.debug.draw_line(
                #         ref_path[i], ref_path[i+1], 
                #         thickness=0.05, color=carla.Color(255, 0, 0), life_time=0.1
                #     )
                
                world.debug.draw_point(
                    car_plot_loc, 
                    size=0.04, 
                    color=carla.Color(0, 255, 0), 
                    life_time=45
                )

                # อัปเดตการบันทึก Log โดยเพิ่มทิศทางต่อท้าย tuple ตัวที่ 7
                green_path_log.append((
                    round(loc.x,      4),
                    round(loc.y,      4),
                    _elapsed,
                    round(_cur_speed, 4),
                    round(_cur_accel, 4),
                    round(_cur_jerk,  4),
                    _direction
                ))
                _prev_speed    = _cur_speed
                _prev_accel    = _cur_accel
                _prev_log_time = _cur_time

                if closest_point:
                    world.debug.draw_line(
                        car_plot_loc, closest_point, 
                        thickness=0.08, color=carla.Color(255, 0, 255), life_time=0.1
                    )
            # =========================================================

            ctrl = carla.VehicleControl()
            ctrl.manual_gear_shift = False

            # === AUTO PARKING STATE MACHINE ===
            if not park_done:
                
                # STATE 1 – drive forward using PID
                if auto_park_state == 1:
                    target_x = PULL_UP_X
                    current_x = loc.x
                    error = target_x - current_x  
                    
                    current_time = time.time()
                    dt = current_time - last_time
                    if dt == 0.0: dt = 0.01 
                    last_time = current_time

                    Kp, Ki, Kd = 0.7, 0.0, 0.7

                    pid_integral += error * dt
                    derivative = (error - pid_prev_error) / dt
                    pid_prev_error = error

                    control_signal = (Kp * error) + (Ki * pid_integral) + (Kd * derivative)

                    vel = car3.get_velocity()
                    speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

                    if error > 0.15:  
                        if control_signal > 0:
                            ctrl.throttle = min(control_signal,0.15)
                            ctrl.brake    = 0.0
                        else:
                            ctrl.throttle = 0.0
                            ctrl.brake    = min(abs(control_signal), 1.0)
                        ctrl.steer = 0.0
                        ctrl.reverse = False
                    else:  
                        ctrl.throttle = 0.0
                        ctrl.brake    = 1.0
                        ctrl.steer    = 0.0
                        if speed < 0.05:
                            set_max_steer(car3, MAX_STEER_ANGLE_STATE2)
                            auto_park_state = 2

                # STATE 2 – reverse arc 1: full RIGHT steer
                elif auto_park_state == 2:
                    ctrl.throttle = THROTTLE_REV
                    ctrl.steer    = STEER_FULL_RIGHT
                    ctrl.brake    = 0.0
                    ctrl.reverse  = True
                    if yaw >= ARC1_YAW_DEG:
                        s5_action_count += 1
                        ctrl.brake    = 1.0
                        current_steer = STEER_FULL_RIGHT
                        set_max_steer(car3, MAX_STEER_ANGLE)
                        auto_park_state = 3

                # STATE 3 – wait and steer to 0
                elif auto_park_state == 3:
                    if state3_timer is None:
                        state3_timer = time.time()  
                    elapsed = time.time() - state3_timer
                    
                    if elapsed < 1.0:
                        ctrl.throttle = 0.0
                        ctrl.brake = 1.0
                        ctrl.steer = current_steer 
                        ctrl.reverse = False
                    else:
                        current_steer = min(current_steer + STEER_UNWIND_STEP, STEER_STRAIGHT)
                        ctrl.throttle = 0.0
                        ctrl.brake = 1.0 
                        ctrl.steer = current_steer
                        ctrl.reverse = False
                        if current_steer >= 0.0:
                            s5_action_count += 1
                            state3_timer = None 
                            auto_park_state = 5

                # STATE 4 – straight backward
                elif auto_park_state == 4:
                    ctrl.steer    = 0.0
                    ctrl.brake    = 0.0
                    ctrl.reverse  = True
                    ctrl.throttle = 0.32
                    physics_control = car3.get_physics_control()
                    phy = physics_control.wheels[3].position.y/100.0
                    phx = physics_control.wheels[3].position.x/100.0
                    # ── หา Y ของกันชนหลัง car3 ในโลก ──────────────────────────
                    half_len = car3.bounding_box.extent.y          # ความยาวครึ่งคัน
                    rear_y   = loc.y - half_len * math.sin(yaw_rad) # rear bumper center Y
                    # ── แนวอ้างอิง = ด้านข้างของ car1/car2 ที่หันหา car3 ──────
                    parked_side_y = car1.get_location().y + car1.bounding_box.extent.y
                    print(f"phy: {phy},parked_side_y: {parked_side_y}")
                    if phy <= parked_side_y:
                        s5_action_count += 1
                        auto_park_state = 5

                # STATE 5 – shunt loop until yaw = 0
                elif auto_park_state == 5:
                    half_len  = car3.bounding_box.extent.x
                    GAP       = 0.035

                    if s5_phase == 'rev':
                        aligned, fully_stopped = check_yaw_aligned(car3, yaw, ctrl)
                        if fully_stopped:
                            auto_park_state = 6
                            park_done       = True
                            print(f"Total Shunting Actions: {s5_action_count}")
                            print("[PARKED] Car fully stopped – permanent handbrake applied.")
                        elif not aligned:
                            ctrl.steer    = 1
                            ctrl.brake    = 0.0
                            ctrl.reverse  = True
                            # Proportional slow-down as yaw approaches 0
                            if abs(yaw) <= YAW_DECEL_ZONE:
                                ctrl.throttle = THROTTLE_REV * (abs(yaw) / 20)
                            else:
                                ctrl.throttle = THROTTLE_REV
                            near_car1 = (loc.x - half_len) <= (car1.get_location().x + half_len + GAP)
                            if near_car1:
                                ctrl.throttle = 0.0
                                ctrl.brake    = 1.0
                                s5_phase      = 'wait_fwd'
                                s5_timer      = time.time()
                            # instant stop if yaw reaches 0 while reversing
                            _, fully_stopped = check_yaw_aligned(car3, yaw, ctrl)
                            if fully_stopped:
                                auto_park_state = 6
                                park_done       = True
                                print(f"Total Shunting Actions: {s5_action_count}")
                                print("[PARKED] Car fully stopped – permanent handbrake applied.")

                    elif s5_phase == 'wait_fwd':
                        aligned, fully_stopped = check_yaw_aligned(car3, yaw, ctrl)
                        if fully_stopped:
                            auto_park_state = 6
                            park_done       = True
                            print(f"Total Shunting Actions: {s5_action_count}")
                            print("[PARKED] Car fully stopped – permanent handbrake applied.")
                        elif not aligned:
                            ctrl.throttle = 0.0
                            ctrl.brake    = 1.0
                            ctrl.steer    = STEER_FULL_LEFT
                            ctrl.reverse  = True
                            if time.time() - s5_timer >= 1.:
                                s5_phase        = 'fwd'
                                s5_action_count += 1
                                s5_timer        = time.time()

                    elif s5_phase == 'fwd':
                        aligned, fully_stopped = check_yaw_aligned(car3, yaw, ctrl)
                        if fully_stopped:
                            auto_park_state = 6
                            park_done       = True
                            print(f"Total Shunting Actions: {s5_action_count}")
                            print("[PARKED] Car fully stopped – permanent handbrake applied.")
                        elif not aligned:
                            ctrl.steer    = STEER_FULL_RIGHT
                            ctrl.brake    = 0.0
                            ctrl.reverse  = False
                            if time.time() - s5_timer >= 1.0:
                                # Proportional slow-down as yaw approaches 0
                                if abs(yaw) <= YAW_DECEL_ZONE:
                                    ctrl.throttle = THROTTLE_FWD * (abs(yaw) / YAW_DECEL_ZONE)
                                else:
                                    ctrl.throttle = THROTTLE_FWD
                            near_car2 = (loc.x + half_len) >= (car2.get_location().x - half_len - GAP)
                            if near_car2 or time.time() - s5_timer >= 3.0:
                                ctrl.throttle = 0.0
                                ctrl.brake    = 1.0
                                s5_phase      = 'wait_rev'
                                s5_timer      = time.time()
                            # instant stop if yaw reaches 0 while driving forward
                            aligned, fully_stopped = check_yaw_aligned(car3, yaw, ctrl)
                            if fully_stopped:
                                auto_park_state = 6
                                park_done       = True
                                print(f"Total Shunting Actions: {s5_action_count}")
                                print("[PARKED] Car fully stopped – permanent handbrake applied.")
                                break

                    elif s5_phase == 'wait_rev':
                        aligned, fully_stopped = check_yaw_aligned(car3, yaw, ctrl)
                        if fully_stopped:
                            auto_park_state = 6
                            park_done       = True
                            print(f"Total Shunting Actions: {s5_action_count}")
                            print("[PARKED] Car fully stopped – permanent handbrake applied.")
                        elif not aligned:
                            ctrl.throttle = 0.0
                            ctrl.brake    = 1.0
                            ctrl.steer    = STEER_FULL_RIGHT
                            ctrl.reverse  = False
                            if time.time() - s5_timer >= 1.0:
                                s5_phase        = 'rev'
                                s5_action_count += 1

                # STATE 6 – DONE
                elif auto_park_state == 6:
                    ctrl.throttle   = 0.0
                    ctrl.brake      = 1.0
                    ctrl.steer      = 0.0
                    ctrl.hand_brake = True
                    park_done       = True
                    print(f"Park Successful! with {s5_action_count} actions")
            
            else:
                ctrl.throttle   = 0.0
                ctrl.brake      = 1.0
                ctrl.steer      = 0.0
                ctrl.hand_brake = True
                
            car3.apply_control(ctrl)

            # === ดึงมุมล้อจริง ณ ขณะนั้น (หน่วยองศา) ===
            actual_steer_deg = car3.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)

            # --- เพิ่มส่วนบันทึกข้อมูลมุมล้อจริงรายเฟรมลงใน Text File Log ---
            _current_elapsed = round(time.time() - _run_start_time, 3)
            steer_log.append([_current_elapsed, round(actual_steer_deg, 4)])
            # --------------------------------------------------------

            # ── Per-frame CSV log ──────────────────────────────────────
            _csv_log.append({
                'frame':        _csv_frame,
                'time_s':       round(time.time() - last_time, 4),
                'state':        auto_park_state,
                's5_phase':     s5_phase if auto_park_state == 5 else '',
                'pos_x':        round(loc.x,  4),
                'pos_y':        round(loc.y,  4),
                'pos_z':        round(loc.z,  4),
                'yaw_deg':      round(yaw,    4),
                'rear_x':       round(rear_x, 4),
                'rear_y':       round(rear_y, 4),
                'speed_ms':     round(_cur_speed, 4),
                'throttle':     round(ctrl.throttle, 4),
                'steer':        round(actual_steer_deg, 4), 
                'brake':        round(ctrl.brake, 4),
                'reverse':      int(ctrl.reverse),
                'hand_brake':   int(ctrl.hand_brake),
                'cte':          round(min_dist, 4) if min_dist != float('inf') else 0,
                'park_done':    int(park_done),
                'direction':    _direction # เพิ่มคอลัมน์ทิศทางลงใน CSV ด้วยเช่นกัน
            })
            _csv_frame += 1
            # ── End log ────────────────────────────────────────────────

            gear_reverse = ctrl.reverse

            render_frame(display, font, car3, ctrl,
                         park_state=auto_park_state,
                         retry_count=retry_count,
                         park_done=park_done,
                         view_mode=view_mode,
                         gear_reverse=gear_reverse,
                         mode_label="AUTONOMOUS PARKING")

    except Exception as e:
        print(f"Error: {e}")
        import traceback; traceback.print_exc()

    finally:
        if green_path_log:
            _end_time   = time.time()
            _total_time = round(_end_time - _run_start_time, 2)
            _start_str  = datetime.datetime.fromtimestamp(_run_start_time).strftime('%Y-%m-%d %H:%M:%S')
            _end_str    = datetime.datetime.fromtimestamp(_end_time).strftime('%Y-%m-%d %H:%M:%S')
            fname = 'green_path_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '.txt'
            with open(fname, 'w') as _f:
                _f.write('# Green Path Log\n')
                _f.write('# Steering : ' + str(MAX_STEER_ANGLE) + ' degrees\n')
                _f.write('# Start time : ' + _start_str + '\n')
                _f.write('# End time   : ' + _end_str + '\n')
                _f.write('# Total time : ' + str(_total_time) + ' seconds\n')
                _f.write('# Points     : ' + str(len(green_path_log)) + '\n')
                # อัปเดตโครงสร้างหัวข้อในคอมเมนต์ไฟล์ TXT
                _f.write('# Format     : [[x, y, elapsed_s, velocity_ms, accel_ms2, jerk_ms3, direction], ...]\n')
                json.dump([[p[0], p[1], p[2], p[3], p[4], p[5], p[6]] for p in green_path_log], _f, indent=2)
            print('[SAVED] Green path (' + str(len(green_path_log)) + ' points, ' + str(_total_time) + 's) -> ' + fname)

        # --- ส่วนบันทึกไฟล์ Steer Log เป็น Text File ---
        if 'steer_log' in locals() and steer_log:
            _end_time   = time.time()
            _total_time = round(_end_time - _run_start_time, 2)
            _start_str  = datetime.datetime.fromtimestamp(_run_start_time).strftime('%Y-%m-%d %H:%M:%S')
            _end_str    = datetime.datetime.fromtimestamp(_end_time).strftime('%Y-%m-%d %H:%M:%S')
            steer_fname = 'steer_log_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '.txt'
            with open(steer_fname, 'w') as _f:
                _f.write('# Steering Wheel Angle Log (Actual Physical Angle)\n')
                _f.write('# Start time : ' + _start_str + '\n')
                _f.write('# End time   : ' + _end_str + '\n')
                _f.write('# Total time : ' + str(_total_time) + ' seconds\n')
                _f.write('# Points     : ' + str(len(steer_log)) + '\n')
                _f.write('# Format     : [[elapsed_seconds, wheel_angle_degrees], ...]\n')
                json.dump(steer_log, _f, indent=2)
            print('[SAVED] Steering log (' + str(len(steer_log)) + ' points) -> ' + steer_fname)

        for actor in actor_list:
            if actor is not None:
                try:
                    actor.destroy()
                except Exception:
                    pass
        pygame.quit()

if __name__ == '__main__':
    main()