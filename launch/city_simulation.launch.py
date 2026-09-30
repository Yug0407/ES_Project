import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    # Set the GZ_SIM_RESOURCE_PATH dynamically to include the models directory
    project_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    models_dir = os.path.join(project_dir, 'models')
    
    set_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_dir
    )
    
    world_file = os.path.join(project_dir, 'worlds', 'city_world.sdf')
    
    # Launch Gazebo Sim with the city world
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    
    return LaunchDescription([
        set_env,
        gz_sim
    ])

if __name__ == '__main__':
    from launch import LaunchService
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
