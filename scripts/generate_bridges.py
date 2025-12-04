#!/usr/bin/env python3
import yaml
import os
import sys
import copy

def generate_bridges_compose(config_path, output_path, base_compose_path="docker-compose.yml"):
    if not os.path.exists(config_path):
        print(f"Config file not found: {config_path}")
        return

    if not os.path.exists(base_compose_path):
        print(f"Base compose file not found: {base_compose_path}")
        return

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    with open(base_compose_path, 'r') as f:
        base_compose = yaml.safe_load(f)

    if not config or 'bridges' not in config:
        print("No bridges defined in config.")
        return

    base_service = base_compose.get('services', {}).get('ros2_bridge')
    if not base_service:
        print("ros2_bridge service not found in base compose file.")
        return

    services = {}
    for bridge in config['bridges']:
        name = bridge.get('name')
        if not name:
            continue

        service_name = f"bridge_{name}"
        
        # Deep copy the base service to avoid mutating it
        service_config = copy.deepcopy(base_service)
        
        # Remove profiles to ensure it starts by default
        if 'profiles' in service_config:
            del service_config['profiles']
            
        # Update container name
        service_config['container_name'] = service_name
        
        # Update environment variables
        # Ensure environment is a dict (it might be a list in some compose files, but here it's a dict)
        if 'environment' not in service_config:
            service_config['environment'] = {}
            
        env = service_config['environment']
        env['ROS_DOMAIN_ID'] = str(bridge.get('ros_domain_id', 43))
        env['ROS1_MASTER_URI'] = bridge.get('ros1_master_uri', '')
        env['ROS1_LOCAL_IP'] = bridge.get('ros1_local_ip', '')
        env['BRIDGE_PARAM_LOADER_NAME'] = f"/ros2_bridge_param_loader_{name}"

        # Optional topics file override
        topics_file = bridge.get('topics_file')
        if topics_file:
             # Mount the custom topics file to the expected location
             if 'volumes' not in service_config:
                 service_config['volumes'] = []
             service_config['volumes'].append(f"{topics_file}:/root/config/ros1_bridge/topics.yaml:ro")

        services[service_name] = service_config

    compose_data = {
        "services": services
    }

    with open(output_path, 'w') as f:
        yaml.dump(compose_data, f, default_flow_style=False)
    
    print(f"Generated {output_path} with {len(services)} bridges.")

if __name__ == "__main__":
    CONFIG_FILE = "config/bridges.yaml"
    OUTPUT_FILE = "docker-compose.bridges.yaml"
    generate_bridges_compose(CONFIG_FILE, OUTPUT_FILE)
