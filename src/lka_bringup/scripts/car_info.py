import carla
import json
import os

def get_car_dimensions():
    vehicle_model = "tesla.model3"
    output_dir = os.path.expanduser('~/park_ws/data/car_info')
    output_file = os.path.join(output_dir, f"{vehicle_model}_dimensions.json")

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(5.0)
        world = client.get_world()

        blueprint_library = world.get_blueprint_library()
        bp = blueprint_library.find(f'vehicle.{vehicle_model}')
        sp = world.get_map().get_spawn_points()[0]
        actor = world.spawn_actor(bp, sp)
 
        world.tick()
        bbox = actor.bounding_box
        physics = actor.get_physics_control()
        wheels = physics.wheels
        
        t = actor.get_transform()
        actor_inv_matrix = t.get_inverse_matrix()
        
        local_wheels = []
        for w in wheels:
             wx = w.position.x / 100.0
             wy = w.position.y / 100.0
             wz = w.position.z / 100.0
             
             lx = actor_inv_matrix[0][0]*wx + actor_inv_matrix[0][1]*wy + actor_inv_matrix[0][2]*wz + actor_inv_matrix[0][3]
             ly = actor_inv_matrix[1][0]*wx + actor_inv_matrix[1][1]*wy + actor_inv_matrix[1][2]*wz + actor_inv_matrix[1][3]
             lz = actor_inv_matrix[2][0]*wx + actor_inv_matrix[2][1]*wy + actor_inv_matrix[2][2]*wz + actor_inv_matrix[2][3]
             local_wheels.append((lx, ly, lz, w))
             
        fl = next(w for w in local_wheels if w[0] > 0 and w[1] < 0)
        fr = next(w for w in local_wheels if w[0] > 0 and w[1] > 0)
        rl = next(w for w in local_wheels if w[0] < 0 and w[1] < 0)
        rr = next(w for w in local_wheels if w[0] < 0 and w[1] > 0)
        a = abs(fl[0] - rl[0]) * 1000  # mm
        track = abs(fl[1] - fr[1]) * 1000  # 2b (mm)
        
        front_axle_x = (fl[0] + fr[0]) / 2.0
        rear_axle_x = (rl[0] + rr[0]) / 2.0
        
        car_front_x = bbox.location.x + bbox.extent.x
        car_rear_x = bbox.location.x - bbox.extent.x
        
        d_front = abs(car_front_x - front_axle_x) * 1000
        d_rear = abs(rear_axle_x - car_rear_x) * 1000
        
        car_left_y = bbox.location.y - bbox.extent.y  # Negative Y side
        car_right_y = bbox.location.y + bbox.extent.y  # Positive Y side
        
        d_l = abs(fl[1] - car_left_y) * 1000
        d_r = abs(car_right_y - fr[1]) * 1000
        
        delta_max = wheels[0].max_steer_angle / 2.0
        
        overall_length = d_front + a + d_rear
        overall_width = d_l + track + d_r

        car_data = {
            "vehicle_model": vehicle_model,
            "dimensions": {
                "overall_length_L_mm": round(overall_length),
                "overall_width_W_mm": round(overall_width),
                "wheelbase_a_mm": round(a),
                "track_2b_mm": round(track),
                "front_overhang_dfront_mm": round(d_front),
                "rear_overhang_drear_mm": round(d_rear),
                "distance_wheel_to_side": {
                    "left_dl_mm": round(d_l),
                    "right_dr_mm": round(d_r)
                },
                "max_steering_angle_deg": round(delta_max)
            }
        }

        os.makedirs(output_dir, exist_ok=True)
        
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(car_data, f, indent=4)
            
        actor.destroy()

    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    get_car_dimensions()