# IR2136_TrabajoFinal 
 
Este repositorio contiene la simulacion de Unity y el ROS package para la realizacion del trabajo.


## Instrucciones para la ejecucion de la simulacion correctamente
Para el correcto funcionamiento se deben seguir los siguientes pasos, en el orden puesto:

    - Iniciar Ardupilot SITL
    - Tener activado el nodo de ROS
    - Ejecutar la simulacion de Unity

### Ardupilot SITL
Para poder abrir Ardupilot SITL se debe ejecutar las siguientes ordenes:

Para acceder a Ardupilot
```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

```
Para instalar las dependencias
```
./Tools/environment_install/install-prereqs-ubuntu.sh -y
```
Para cargar las configuraciones
```
. ~/.profile
```
Para abrir SITL
```
cd ArduCopter   (dentro de ardupilot/)
sim_vehicle.py -v ArduCopter --map --console  --out=udp:127.0.0.1:14550
```
### ROS2 Node
Para lanzar el Nodo de ROS2 se deben realizar los siguientes pasos:

Primeramente colocarse en el directorio donde esta el workspace. En mi caso
```
cd /home/joel/Documentos/IR2136_TrabajoFinal-/mavlink
```
Seguidamente compilamos el proyecto
```
source /opt/ros/humble/setup.bash (Si se tiene Ubuntu 24.04 se sustituye humble por jazzy)
colcon build
source install/setup/bash
```
Por último lanzamos el nodo de ROS
```
ros2 run pymavlink_ros manage_unity
```

### Unity
1) En primer lugar, abrimos el proyecto "My project" que contiene la simulación
2) En segundo lugar le damos al play para poder interactuar 
3) Para mover el dron debemos utilizar las siguientes teclas:
    - A --> para avanzar hacia la izquierda
    - W --> para avanzar hacia delante
    - S --> para avanzar hacia atrás
    - D --> para avanzar hacia la derecha
    - Q --> para elevar el dron
    - E --> para descender el dron

4) Guardar las posiciones:

Una vez se haya colocado el dron en el punto deseado, se pulsa la tecla "Space" para almacenar la posicion actual.

Cuando se hayan almacenado todas las posiciones, se debe pulsar la tecla "F" para indicar que se ha completado la ruta y se publique dicha ruta.


Una vez se ha pulsado la tecla "F", actuara el codigo del nodo de ROS2, que estaba esperando a que se publicara la lista. Una vez publicada, el dron se armará, cambiará a modo GUIDED y se elevara unos 10m (en ocasiones puede dar error de armado, por tanto repetir el proceso) y avanzara hasta el primer punto.