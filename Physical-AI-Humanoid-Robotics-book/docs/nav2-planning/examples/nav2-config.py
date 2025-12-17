#!/usr/bin/env python3
"""
Nav2 Configuration for Humanoid Robots
Demonstrates how to configure Nav2 for humanoid robot navigation
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from rclpy.duration import Duration as RCLDuration
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import yaml
import json


class Nav2HumanoidConfigurator(Node):
    """
    Node to demonstrate Nav2 configuration for humanoid robots
    """

    def __init__(self):
        super().__init__('nav2_humanoid_configurator')

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF buffer and listener for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Store configuration parameters
        self.nav2_params = self.get_default_humanoid_nav2_params()

        # Publishers and subscribers
        self.config_publisher = self.create_publisher(
            # In a real implementation, this would be a custom message type
            # For now, we'll just log the configuration
        )

        self.get_logger().info("Nav2 Humanoid Configurator initialized")

    def get_default_humanoid_nav2_params(self):
        """
        Get default Nav2 parameters optimized for humanoid robots
        """
        params = {
            # Global Planner Configuration
            'global_costmap': {
                'global_costmap': {
                    'plugins': ['obstacle_layer', 'inflation_layer'],
                    'obstacle_layer': {
                        'plugin': 'nav2_costmap_2d::ObstacleLayer',
                        'enabled': True,
                        'observation_sources': 'scan',
                        'scan': {
                            'topic': '/scan',
                            'max_obstacle_height': 2.0,
                            'raytrace_range': 3.0,
                            'obstacle_range': 2.5,
                            'marking': True,
                            'clearing': True
                        }
                    },
                    'inflation_layer': {
                        'plugin': 'nav2_costmap_2d::InflationLayer',
                        'cost_scaling_factor': 3.0,
                        'inflation_radius': 1.0
                    },
                    'always_send_full_costmap': True,
                    'update_frequency': 1.0,
                    'publish_frequency': 1.0,
                    'width': 20,
                    'height': 20,
                    'resolution': 0.05,
                    'origin_x': -10.0,
                    'origin_y': -10.0
                }
            },

            # Local Planner Configuration (Humanoid-specific)
            'local_costmap': {
                'local_costmap': {
                    'plugins': ['obstacle_layer', 'voxel_layer', 'inflation_layer'],
                    'obstacle_layer': {
                        'plugin': 'nav2_costmap_2d::ObstacleLayer',
                        'enabled': True,
                        'observation_sources': 'scan',
                        'scan': {
                            'topic': '/scan',
                            'max_obstacle_height': 2.0,
                            'raytrace_range': 2.0,
                            'obstacle_range': 1.5,
                            'marking': True,
                            'clearing': True
                        }
                    },
                    'voxel_layer': {
                        'plugin': 'nav2_costmap_2d::VoxelLayer',
                        'enabled': True,
                        'max_obstacle_height': 2.0,
                        'origin_z': 0.0,
                        'z_voxels': 16,
                        'z_resolution': 0.05
                    },
                    'inflation_layer': {
                        'plugin': 'nav2_costmap_2d::InflationLayer',
                        'cost_scaling_factor': 3.0,
                        'inflation_radius': 0.5
                    },
                    'always_send_full_costmap': False,
                    'update_frequency': 5.0,
                    'publish_frequency': 2.0,
                    'width': 6,
                    'height': 6,
                    'resolution': 0.05,
                    'origin_x': -3.0,
                    'origin_y': -3.0
                }
            },

            # Controller Server Configuration (Humanoid-specific)
            'controller_server': {
                'controller_server': {
                    'plugin_names': ['humanoid_controller'],
                    'humanoid_controller': {
                        'plugin': 'nav2_mppi_controller::MppiController',
                        'time_steps': 20,
                        'control_horizon': 10,
                        'model_dt': 0.1,
                        'xy_goal_tolerance': 0.25,
                        'yaw_goal_tolerance': 0.25,
                        'stateful': True,
                        'publish_cost_grid': False,
                        'speed_regulation_factor': 0.1,
                        'obstacle_cost_weight': 1.0,
                        'goal_cost_weight': 1.0,
                        'control_cost_weight': 0.0,
                        'curvature_cost_weight': 0.0,
                        # Humanoid-specific parameters
                        'max_humanoid_vel_x': 0.3,  # Slower for stability
                        'min_humanoid_vel_x': -0.1,
                        'max_humanoid_vel_theta': 0.5,  # Limited turning
                        'max_humanoid_acc_x': 0.5,
                        'max_humanoid_acc_theta': 0.5,
                        'step_size_limit': 0.3,  # Max step size
                        'foot_separation': 0.2,  # Distance between feet
                        'stance_time': 0.1  # Time for stable stance
                    }
                },
                'progress_checker_plugin': 'progress_checker',
                'goal_checker_plugin': 'goal_checker',
                'controller_frequency': 10.0,  # Lower for humanoid stability
                'min_x_velocity_threshold': 0.01,
                'min_y_velocity_threshold': 0.01,
                'min_theta_velocity_threshold': 0.01,
                'failure_tolerance': 0.3
            },

            # Planner Server Configuration
            'planner_server': {
                'planner_server': {
                    'plugin_names': ['GridBased'],
                    'GridBased': {
                        'plugin': 'nav2_navfn_planner::NavfnPlanner',
                        'tolerance': 0.5,
                        'use_astar': False,
                        'allow_unknown': True,
                        # Humanoid-specific parameters
                        'humanoid_step_cost_multiplier': 1.5,
                        'balance_penalty': 0.5
                    }
                },
                'planner_frequency': 1.0,  # Lower planning frequency for humanoid
                'expected_planner_frequency': 1.0
            },

            # Behavior Server Configuration
            'behavior_server': {
                'local_costmap': 'local_costmap',
                'global_costmap': 'global_costmap',
                'behavior_tree_xml_filename': 'humanoid_navigate_w_replanning_and_recovery.xml',
                'costmap_topic': 'local_costmap/costmap_raw',
                'footprint_topic': 'local_costmap/published_footprint',
                'cycle_frequency': 10.0,
                'behavior_names': [
                    'spin',
                    'backup',
                    'drive_on_heading',
                    'assisted_teleop',
                    'wait'
                ],
                'spin': {
                    'plugin': 'nav2_behaviors::Spin'
                },
                'backup': {
                    'plugin': 'nav2_behaviors::BackUp'
                },
                'drive_on_heading': {
                    'plugin': 'nav2_behaviors::DriveOnHeading'
                },
                'assisted_teleop': {
                    'plugin': 'nav2_behaviors::AssistedTeleop'
                },
                'wait': {
                    'plugin': 'nav2_behaviors::Wait'
                },
                'global_frame': 'map',
                'robot_base_frame': 'base_link',
                'transform_tolerance': 0.1,
                'use_sim_time': True
            },

            # Waypoint Follower Configuration
            'waypoint_follower': {
                'waypoint_follower': {
                    'loop_rate': 20,
                    'stop_on_failure': True,
                    'goal_check_tolerance': 0.25,
                    'humanoid_specific': {
                        'max_step_count_per_waypoint': 50,
                        'balance_check_frequency': 5.0
                    }
                }
            }
        }

        return params

    def save_nav2_config(self, config_path="humanoid_nav2_config.yaml"):
        """
        Save the Nav2 configuration to a YAML file
        """
        try:
            # Create a simplified version for the example
            config_dict = {
                'bt_navigator': {
                    'ros__parameters': {
                        'use_sim_time': True,
                        'global_frame': 'map',
                        'robot_base_frame': 'base_link',
                        'odom_topic': '/odom',
                        'default_bt_xml_filename': 'humanoid_navigate_w_replanning_and_recovery.xml',
                        'plugin_lib_names': [
                            'nav2_compute_path_to_pose_action_bt_node',
                            'nav2_follow_path_action_bt_node',
                            'nav2_back_up_action_bt_node',
                            'nav2_spin_action_bt_node',
                            'nav2_wait_action_bt_node',
                            'nav2_clear_costmap_service_bt_node',
                            'nav2_is_stuck_condition_bt_node',
                            'nav2_goal_reached_condition_bt_node',
                            'nav2_goal_updated_condition_bt_node',
                            'nav2_initial_pose_received_condition_bt_node',
                            'nav2_reinitialize_global_localization_service_bt_node',
                            'nav2_rate_controller_bt_node',
                            'nav2_distance_controller_bt_node',
                            'nav2_speed_controller_bt_node',
                            'nav2_truncate_path_action_bt_node',
                            'nav2_goal_updater_node_bt_node',
                            'nav2_recovery_node_bt_node',
                            'nav2_pipeline_sequence_bt_node',
                            'nav2_round_robin_node_bt_node',
                            'nav2_transform_available_condition_bt_node',
                            'nav2_time_expired_condition_bt_node',
                            'nav2_path_expiring_timer_condition',
                            'nav2_distance_traveled_condition_bt_node',
                            'nav2_single_trigger_bt_node',
                            'nav2_is_battery_low_condition_bt_node',
                            'nav2_navigate_through_poses_action_bt_node',
                            'nav2_navigate_to_pose_action_bt_node',
                            'nav2_remove_passed_goals_action_bt_node',
                            'nav2_planner_selector_bt_node',
                            'nav2_controller_selector_bt_node',
                            'nav2_goal_checker_selector_bt_node',
                            'nav2_controller_cancel_bt_node',
                            'nav2_path_longer_on_approach_bt_node',
                            'nav2_wait_cancel_bt_node',
                            'nav2_spin_cancel_bt_node',
                            'nav2_backup_cancel_bt_node'
                        ]
                    }
                },
                'controller_server': {
                    'ros__parameters': self.nav2_params['controller_server']['controller_server']
                },
                'planner_server': {
                    'ros__parameters': self.nav2_params['planner_server']['planner_server']
                },
                'recoveries_server': {
                    'ros__parameters': {
                        'costmap_topic': 'local_costmap/costmap_raw',
                        'footprint_topic': 'local_costmap/published_footprint',
                        'cycle_frequency': 10.0,
                        'global_frame': 'map',
                        'robot_base_frame': 'base_link',
                        'transform_timeout': 0.1,
                        'use_sim_time': True,
                        'recovery_plugins': ['spin', 'backup', 'wait'],
                        'spin': {
                            'plugin': 'nav2_recoveries::Spin'
                        },
                        'backup': {
                            'plugin': 'nav2_recoveries::BackUp'
                        },
                        'wait': {
                            'plugin': 'nav2_recoveries::Wait'
                        }
                    }
                },
                'local_costmap': {
                    'ros__parameters': self.nav2_params['local_costmap']['local_costmap']
                },
                'global_costmap': {
                    'ros__parameters': self.nav2_params['global_costmap']['global_costmap']
                }
            }

            with open(config_path, 'w') as config_file:
                yaml.dump(config_dict, config_file, default_flow_style=False)

            self.get_logger().info(f"Nav2 configuration saved to {config_path}")
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to save Nav2 configuration: {e}")
            return False

    def load_nav2_config(self, config_path="humanoid_nav2_config.yaml"):
        """
        Load Nav2 configuration from a YAML file
        """
        try:
            with open(config_path, 'r') as config_file:
                config = yaml.safe_load(config_file)

            self.get_logger().info(f"Nav2 configuration loaded from {config_path}")
            return config

        except Exception as e:
            self.get_logger().error(f"Failed to load Nav2 configuration: {e}")
            return None

    def configure_humanoid_navigation(self):
        """
        Configure Nav2 specifically for humanoid robot navigation
        """
        self.get_logger().info("Configuring Nav2 for humanoid robot navigation...")

        # Save the configuration
        config_saved = self.save_nav2_config()

        if config_saved:
            self.get_logger().info("Nav2 configuration for humanoid robots completed successfully")
            return True
        else:
            self.get_logger().error("Failed to configure Nav2 for humanoid robots")
            return False

    def get_humanoid_behavior_tree(self):
        """
        Get a sample behavior tree XML for humanoid navigation
        """
        bt_xml = """<?xml version="1.0" encoding="UTF-8"?>
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <ReactiveSequence>
            <GoalUpdated/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
            <ReactiveFallback name="RecoveryFallback">
                <PipelineSequence name="ComputeAndFollowPath">
                    <RateController hz="10">
                        <ComputePathToPose goal="current_goal" path="path"/>
                    </RateController>
                    <FollowPath path="path"/>
                </PipelineSequence>
                <ReactiveSequence>
                    <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
                    <ReactiveFallback name="RecoveryFallback-1">
                        <Spin spin_dist="1.57"/>
                        <BackUp backup_dist="0.15" backup_speed="0.05"/>
                        <Wait wait_duration="5"/>
                    </ReactiveFallback>
                </ReactiveSequence>
            </ReactiveFallback>
        </ReactiveSequence>
    </BehaviorTree>
</root>"""

        return bt_xml

    def save_behavior_tree(self, bt_path="humanoid_navigate_w_replanning_and_recovery.xml"):
        """
        Save the humanoid-specific behavior tree
        """
        try:
            bt_content = self.get_humanoid_behavior_tree()
            with open(bt_path, 'w') as bt_file:
                bt_file.write(bt_content)

            self.get_logger().info(f"Humanoid behavior tree saved to {bt_path}")
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to save behavior tree: {e}")
            return False


def main(args=None):
    """
    Main function to run the Nav2 configuration example
    """
    rclpy.init(args=args)

    configurator = Nav2HumanoidConfigurator()

    try:
        # Configure Nav2 for humanoid navigation
        success = configurator.configure_humanoid_navigation()

        if success:
            # Save the behavior tree
            configurator.save_behavior_tree()

            configurator.get_logger().info("Nav2 configuration for humanoid robots completed successfully!")
        else:
            configurator.get_logger().error("Failed to configure Nav2 for humanoid robots")

    except Exception as e:
        configurator.get_logger().error(f"Error during configuration: {e}")

    finally:
        configurator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()