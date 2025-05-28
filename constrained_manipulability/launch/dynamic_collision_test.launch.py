# from launch import LaunchDescription
# from launch.actions import ExecuteProcess, TimerAction
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         # 1. Lanzar URSim
#         ExecuteProcess(
#             cmd=['ros2', 'run', 'ur_client_library', 'start_ursim.sh', '-m', 'ur10e'],
#             shell=True,
#             output='screen'
#         ),

#         # 2. Esperar 5 segundos
#         TimerAction(
#             period=5.0,
#             actions=[
#                 # 3. Lanzar controlador del UR10e en modo fake
#                 ExecuteProcess(
#                     cmd=[
#                         'ros2', 'launch', 'ur_arm_control', 'ur_control.launch.py',
#                         'ur_type:=ur10e', 'robot_ip:=192.168.56.101',
#                         'use_fake_hardware:=true', 'launch_rviz:=true'
#                     ],
#                     shell=True,
#                     output='screen'
#                 )
#             ]
#         ),

#         # 4. Esperar otros 5 segundos para que esté disponible robot_description
#         TimerAction(
#             period=15.0,
#             actions=[
#                 # 5. Lanzar tu nodo de colisiones
#                 Node(
#                     package='constrained_manipulability',
#                     executable='dynamic_collision_world_node',
#                     name='dynamic_collision_world_node',
#                     output='screen'
#                 )
#             ]
#         )
#     ])
