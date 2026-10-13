## Ejecutar

```bash
# Terminal 1: Gazebo + robot + bridge + cinemática + RViz
ros2 launch mi_robot_mecanum simulacion.launch.py

# Terminal 2: prueba con teclado (Shift + J / L = desplazamiento lateral)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 2: control a puntos
ros2 run mi_mecanum_control ir_a_puntos --ros-args -p use_sim_time:=true
