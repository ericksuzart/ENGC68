# Como rodar o mundo

# Build

No nível dessa pasta, rode

    catkin build
    source devel/setup.bash
    roscore

## Iniciar gazebo

    roslaunch lar_gazebo lar_world.launch

## Iniciar Rviz

    roslaunch husky_viz view_robot.launch 

## Rodar script em python

    source devel/setup.bash
    rosrun comparison comparer.py
    
## Exemplo de comando para movimentar Husky

```
rostopic pub /husky_velocity_controller/cmd_vel geometry_msgs/Twist "linear:
        x: 0.0
        y: 1.0
        z: 0.0
angular:
        x: 0.0
        y: 0.0
        z: -0.8" -r 10
```

## Exemplo de comando para reposicionar Husky

```
rosservice call /gazebo/set_model_state '{model_state: {model_name: husky, pose: { position: { x: -4, y: 0.5, z: 0}, orientation: { x: 0, y: 0.0, z: 0, w: 0 } }, twist: { linear: { x: 0.0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } }, reference_frame: world } }'
```