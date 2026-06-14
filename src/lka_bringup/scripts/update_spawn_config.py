#!/usr/bin/env python3
import json
import os
import sys

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 update_spawn_config.py <L_value>")
        sys.exit(1)
        
    try:
        L = float(sys.argv[1])
    except ValueError:
        print("Error: L must be a valid number.")
        sys.exit(1)
        
    json_file = os.path.expanduser('~/park_ws/src/lka_bringup/config/objects.json')
    
    with open(json_file, 'r') as f:
        data = json.load(f)
        
    updated = False
    for obj in data.get('objects', []):
        if obj.get('id') == 'obstacle_front':
            new_x = 69.8 + L
            obj['spawn_point']['x'] = round(new_x, 3)
            updated = True
            print(f"Success: Updated 'obstacle_front' x spawn_point to {new_x:.3f} (L={L})")
            break
            
    with open(json_file, 'w') as f:
        json.dump(data, f, indent=4)

if __name__ == '__main__':
    main()