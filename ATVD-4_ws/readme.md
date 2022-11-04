# Como rodar o mundo

# Build

No nível dessa pasta, rode

    catkin build

## Iniciar gazebo

    roslaunch lar_gazebo lar_world.launch

## Iniciar Rviz

    roslaunch husky_viz view_robot.launch    

## Rodar script em python

    source devel/setup.bash
    rosrun comparison comparer.py