# Como rodar o mundo

# Build

No nível dessa pasta, rode

    catkin build
    roscore

## Iniciar gazebo

    roslaunch lar_gazebo lar_world.launch

## Rodar script em python

    rosrun point_stabilizer stabilizer.py x y

    (Exemplo:) rosrun point_stabilizer stabilizer.py 0 0