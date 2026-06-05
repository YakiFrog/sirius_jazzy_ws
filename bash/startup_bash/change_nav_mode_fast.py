#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class ParameterSetter(Node):
    def __init__(self):
        super().__init__('parameter_setter_node')
        
    def set_node_parameters(self, node_name, params_dict):
        srv_name = f'{node_name}/set_parameters'
        client = self.create_client(SetParameters, srv_name)
        if not client.wait_for_service(timeout_sec=5.0):
            print(f"[Warning] Node '{node_name}' is not running or service not available.")
            return False
            
        req = SetParameters.Request()
        for name, val in params_dict.items():
            param = Parameter()
            param.name = name
            
            p_val = ParameterValue()
            if isinstance(val, bool):
                p_val.type = ParameterType.PARAMETER_BOOL
                p_val.bool_value = val
            elif isinstance(val, int):
                p_val.type = ParameterType.PARAMETER_INTEGER
                p_val.integer_value = val
            elif isinstance(val, float):
                p_val.type = ParameterType.PARAMETER_DOUBLE
                p_val.double_value = val
            elif isinstance(val, str):
                p_val.type = ParameterType.PARAMETER_STRING
                p_val.string_value = val
            elif isinstance(val, list):
                if len(val) > 0 and isinstance(val[0], float):
                    p_val.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                    p_val.double_array_value = val
                elif len(val) > 0 and isinstance(val[0], int):
                    p_val.type = ParameterType.PARAMETER_INTEGER_ARRAY
                    p_val.integer_array_value = val
                else:
                    p_val.type = ParameterType.PARAMETER_STRING_ARRAY
                    p_val.string_array_value = val
            
            param.value = p_val
            req.parameters.append(param)
            
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done():
            res = future.result()
            all_ok = True
            for r in res.results:
                if not r.successful:
                    print(f"Failed to set parameter '{name}' on {node_name}: {r.reason}")
                    all_ok = False
            return all_ok
        else:
            print(f"Timeout setting parameters on {node_name}")
            return False

def main():
    if len(sys.argv) < 2:
        print("Usage: change_nav_mode_fast.py <mode>")
        sys.exit(1)
        
    mode = sys.argv[1]
    
    configs = {}
    
    # 1. normal
    configs['normal'] = {
        '/controller_server': {
            'FollowPath.vx_max': 0.90,
            'FollowPath.vx_min': -0.60,
            'FollowPath.wz_max': 0.90,
            'FollowPath.vx_std': 0.25,
            'FollowPath.wz_std': 0.30,
            'FollowPath.ax_max': 0.90,
            'FollowPath.ax_min': -0.90,
            'FollowPath.az_max': 1.50,
            'FollowPath.CostCritic.cost_weight': 10.0,
            'FollowPath.PathAlignCritic.cost_weight': 8.0,
            'FollowPath.PathFollowCritic.cost_weight': 8.0,
            'FollowPath.PathAngleCritic.cost_weight': 2.0,
            'FollowPath.TwirlingCritic.cost_weight': 3.0,
            'FollowPath.PreferForwardCritic.cost_weight': 15.0,
            'FollowPath.GoalCritic.cost_weight': 3.0,
            'FollowPath.GoalAngleCritic.cost_weight': 1.0,
        },
        '/velocity_smoother': {
            'max_velocity': [0.90, 0.0, 0.90],
            'min_velocity': [-0.90, 0.0, -0.90],
            'max_accel': [0.90, 0.0, 1.50],
            'max_decel': [-0.90, 0.0, -1.50]
        },
        '/global_costmap/global_costmap': {
            'obstacle_layer.enabled': True
        }
    }
    
    # 2. normal_active
    configs['normal_active'] = {
        '/controller_server': {
            'FollowPath.vx_max': 1.00,
            'FollowPath.vx_min': -0.60,
            'FollowPath.wz_max': 1.00,
            'FollowPath.vx_std': 0.40,
            'FollowPath.wz_std': 0.48,
            'FollowPath.ax_max': 1.50,
            'FollowPath.ax_min': -1.50,
            'FollowPath.az_max': 2.20,
            'FollowPath.CostCritic.cost_weight': 20.0,
            'FollowPath.PathAlignCritic.cost_weight': 8.0,
            'FollowPath.PathFollowCritic.cost_weight': 8.0,
            'FollowPath.PathAngleCritic.cost_weight': 2.0,
            'FollowPath.TwirlingCritic.cost_weight': 3.0,
            'FollowPath.PreferForwardCritic.cost_weight': 15.0,
            'FollowPath.GoalCritic.cost_weight': 3.0,
            'FollowPath.GoalAngleCritic.cost_weight': 1.0,
        },
        '/velocity_smoother': {
            'max_velocity': [1.00, 0.0, 1.00],
            'min_velocity': [-0.60, 0.0, -1.00],
            'max_accel': [1.50, 0.0, 2.20],
            'max_decel': [-1.50, 0.0, -2.20]
        },
        '/global_costmap/global_costmap': {
            'obstacle_layer.enabled': True
        }
    }
    
    # 3. safe
    configs['safe'] = {
        '/controller_server': {
            'FollowPath.vx_max': 0.40,
            'FollowPath.vx_min': -0.20,
            'FollowPath.wz_max': 0.40,
            'FollowPath.vx_std': 0.20,
            'FollowPath.wz_std': 0.20,
            'FollowPath.ax_max': 0.40,
            'FollowPath.ax_min': -0.40,
            'FollowPath.az_max': 1.00,
            'FollowPath.CostCritic.cost_weight': 15.0,
            'FollowPath.PathAlignCritic.cost_weight': 8.0,
            'FollowPath.PathFollowCritic.cost_weight': 8.0,
            'FollowPath.PathAngleCritic.cost_weight': 2.0,
            'FollowPath.TwirlingCritic.cost_weight': 3.0,
            'FollowPath.PreferForwardCritic.cost_weight': 15.0,
            'FollowPath.GoalCritic.cost_weight': 3.0,
            'FollowPath.GoalAngleCritic.cost_weight': 1.0,
        },
        '/velocity_smoother': {
            'max_velocity': [0.40, 0.0, 0.40],
            'min_velocity': [-0.20, 0.0, -0.40],
            'max_accel': [0.40, 0.0, 1.00],
            'max_decel': [-0.40, 0.0, -1.00]
        },
        '/global_costmap/global_costmap': {
            'obstacle_layer.enabled': True
        }
    }
    
    # 4. slow
    configs['slow'] = {
        '/controller_server': {
            'FollowPath.vx_max': 0.20,
            'FollowPath.vx_min': -0.10,
            'FollowPath.wz_max': 0.20,
            'FollowPath.vx_std': 0.20,
            'FollowPath.wz_std': 0.20,
            'FollowPath.ax_max': 0.20,
            'FollowPath.ax_min': -0.20,
            'FollowPath.az_max': 0.50,
            'FollowPath.CostCritic.cost_weight': 20.0,
            'FollowPath.PathAlignCritic.cost_weight': 8.0,
            'FollowPath.PathFollowCritic.cost_weight': 8.0,
            'FollowPath.PathAngleCritic.cost_weight': 2.0,
            'FollowPath.TwirlingCritic.cost_weight': 3.0,
            'FollowPath.PreferForwardCritic.cost_weight': 15.0,
            'FollowPath.GoalCritic.cost_weight': 3.0,
            'FollowPath.GoalAngleCritic.cost_weight': 1.0,
        },
        '/velocity_smoother': {
            'max_velocity': [0.20, 0.0, 0.20],
            'min_velocity': [-0.10, 0.0, -0.20],
            'max_accel': [0.20, 0.0, 0.50],
            'max_decel': [-0.20, 0.0, -0.50]
        },
        '/global_costmap/global_costmap': {
            'obstacle_layer.enabled': True
        }
    }
    
    # 5. strict_normal
    configs['strict_normal'] = {
        '/controller_server': {
            'FollowPath.vx_max': 0.90,
            'FollowPath.vx_min': -0.60,
            'FollowPath.wz_max': 0.90,
            'FollowPath.vx_std': 0.25,
            'FollowPath.wz_std': 0.30,
            'FollowPath.ax_max': 0.90,
            'FollowPath.ax_min': -0.90,
            'FollowPath.az_max': 1.50,
            'FollowPath.CostCritic.cost_weight': 15.0,
            'FollowPath.PathAlignCritic.cost_weight': 60.0,
            'FollowPath.PathFollowCritic.cost_weight': 3.0,
            'FollowPath.PathAngleCritic.cost_weight': 1.5,
            'FollowPath.TwirlingCritic.cost_weight': 5.0,
            'FollowPath.PreferForwardCritic.cost_weight': 25.0,
            'FollowPath.GoalCritic.cost_weight': 0.5,
            'FollowPath.GoalAngleCritic.cost_weight': 0.5,
        },
        '/velocity_smoother': {
            'max_velocity': [0.90, 0.0, 0.90],
            'min_velocity': [-0.60, 0.0, -0.90],
            'max_accel': [0.90, 0.0, 1.50],
            'max_decel': [-0.90, 0.0, -1.50]
        },
        '/global_costmap/global_costmap': {
            'obstacle_layer.enabled': False
        }
    }
    
    # 6. strict_safe
    configs['strict_safe'] = {
        '/controller_server': {
            'FollowPath.vx_max': 0.40,
            'FollowPath.vx_min': -0.20,
            'FollowPath.wz_max': 0.40,
            'FollowPath.vx_std': 0.20,
            'FollowPath.wz_std': 0.20,
            'FollowPath.ax_max': 0.40,
            'FollowPath.ax_min': -0.40,
            'FollowPath.az_max': 1.00,
            'FollowPath.CostCritic.cost_weight': 15.0,
            'FollowPath.PathAlignCritic.cost_weight': 60.0,
            'FollowPath.PathFollowCritic.cost_weight': 3.0,
            'FollowPath.PathAngleCritic.cost_weight': 1.5,
            'FollowPath.TwirlingCritic.cost_weight': 5.0,
            'FollowPath.PreferForwardCritic.cost_weight': 25.0,
            'FollowPath.GoalCritic.cost_weight': 0.5,
            'FollowPath.GoalAngleCritic.cost_weight': 0.5,
        },
        '/velocity_smoother': {
            'max_velocity': [0.40, 0.0, 0.40],
            'min_velocity': [-0.20, 0.0, -0.40],
            'max_accel': [0.40, 0.0, 1.00],
            'max_decel': [-0.40, 0.0, -1.00]
        },
        '/global_costmap/global_costmap': {
            'obstacle_layer.enabled': False
        }
    }
    
    # 7. strict_slow
    configs['strict_slow'] = {
        '/controller_server': {
            'FollowPath.vx_max': 0.20,
            'FollowPath.vx_min': -0.10,
            'FollowPath.wz_max': 0.20,
            'FollowPath.vx_std': 0.20,
            'FollowPath.wz_std': 0.20,
            'FollowPath.ax_max': 0.20,
            'FollowPath.ax_min': -0.20,
            'FollowPath.az_max': 0.50,
            'FollowPath.CostCritic.cost_weight': 20.0,
            'FollowPath.PathAlignCritic.cost_weight': 60.0,
            'FollowPath.PathFollowCritic.cost_weight': 3.0,
            'FollowPath.PathAngleCritic.cost_weight': 1.5,
            'FollowPath.TwirlingCritic.cost_weight': 5.0,
            'FollowPath.PreferForwardCritic.cost_weight': 25.0,
            'FollowPath.GoalCritic.cost_weight': 0.5,
            'FollowPath.GoalAngleCritic.cost_weight': 0.5,
        },
        '/velocity_smoother': {
            'max_velocity': [0.20, 0.0, 0.20],
            'min_velocity': [-0.10, 0.0, -0.20],
            'max_accel': [0.20, 0.0, 0.50],
            'max_decel': [-0.20, 0.0, -0.50]
        },
        '/global_costmap/global_costmap': {
            'obstacle_layer.enabled': False
        }
    }
    
    if mode not in configs:
        print(f"Error: Mode '{mode}' is not defined.")
        sys.exit(1)
        
    rclpy.init()
    node = ParameterSetter()
    
    config = configs[mode]
    success = True
    for node_name, params in config.items():
        print(f"Setting parameters on '{node_name}'...")
        if not node.set_node_parameters(node_name, params):
            success = False
            
    node.destroy_node()
    rclpy.shutdown()
    
    if success:
        print("✓ Successfully applied all navigation parameters!")
        sys.exit(0)
    else:
        print("⚠ Some parameters failed to apply.")
        sys.exit(1)

if __name__ == '__main__':
    main()
