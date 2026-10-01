ip -4 addr show scope global          # debe salir una IP del laboratorio (no 169.254...)

JETSON=<IP_del_Jetson_del_equipo_5>   # se la pide al profe
ping -c 3 $JETSON
echo "export JETSON=$JETSON" >> ~/.bashrc

echo "$ROS_DISTRO · dominio $ROS_DOMAIN_ID · localhost_only $ROS_LOCALHOST_ONLY · server $ROS_DISCOVERY_SERVER"
ros2 daemon stop && ros2 daemon start


PASO 1
ip -4 addr show scope global          # debe salir una IP del laboratorio (no 169.254...)

JETSON=<IP_del_Jetson_del_equipo_5>   # se la pide al profe
ping -c 3 $JETSON
echo "export JETSON=$JETSON" >> ~/.bashrc
PASO 2

sudo apt update
sudo apt install -y ros-humble-rosidl-default-generators ros-humble-rosbag2 \
  ros-humble-rosbag2-storage-default-plugins ros-humble-sensor-msgs \
  ros-humble-demo-nodes-cpp ros-humble-turtlesim ros-humble-rmw-fastrtps-cpp \
  python3-colcon-common-extensions python3-matplotlib

PASO 3
scp jetson@$JETSON:~/super_client_configuration_file.xml ~/

echo 'source /opt/ros/humble/setup.bash'                                   >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=47'                                             >> ~/.bashrc
echo 'export ROS_LOCALHOST_ONLY=0'                                         >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp'                          >> ~/.bashrc
echo 'export ROS_DISCOVERY_SERVER=$JETSON:11811'                           >> ~/.bashrc
echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/super_client_configuration_file.xml' >> ~/.bashrc
exec bash

echo "$ROS_DISTRO · dominio $ROS_DOMAIN_ID · localhost_only $ROS_LOCALHOST_ONLY · server $ROS_DISCOVERY_SERVER"
ros2 daemon stop && ros2 daemon start



########
ros2 daemon stop; ros2 daemon start
ros2 node list
ros-brazo          # el comentario del bloque dice que muestra el estado
###########

ls -l ~/super_client_configuration_file.xml
grep -n -E "172\.|11811" ~/super_client_configuration_file.xml
echo ------
grep -n -E "172\.|11811" /var/lib/ros-brazo/super_client.xml
cat /var/lib/ros-brazo/ros.env
ip -4 addr show scope global | grep inet


-rw-r--r-- 1 alumno01 alumno01 952 Sep 24 15:48 /home/alumno01/super_client_configuration_file.xml
14:                  <locator><udpv4><address>127.0.0.1</address><port>11811</port></udpv4></locator>
------
14:                  <locator><udpv4><address>172.51.9.13</address><port>11811</port></udpv4></locator>
# Generado por ros-brazo (2026-09-24 09:12:55): brazo jetson-arm-01
export ROS_BRAZO=jetson-arm-01
export ROS_DOMAIN_ID=52
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=172.51.9.13:11811
export FASTRTPS_DEFAULT_PROFILES_FILE=/var/lib/ros-brazo/super_client.xml
    inet 172.51.9.23/16 brd 172.51.255.255 scope global dynamic noprefixroute wlan0
alumno01@rpi-19:~$ 

c

 alumno01@rpi-19:~$ grep -n -E "172\.|11811" ~/super_client_jetson118.xml
14:                  <locator><udpv4><address>127.0.0.1</address><port>11811</port></udpv4></locator>

