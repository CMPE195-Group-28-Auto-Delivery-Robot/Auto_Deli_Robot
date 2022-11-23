apt install gpsd gpsd-clients python-gps -y
apt-get install ros-melodic-nmea-navsat-driver -y   
apt-get install ros-melodic-robot-localization -y
apt-get install ros-melodic-mapviz -y
apt install ros-melodic-mapviz-plugins -y
apt install ros-melodic-tile-map -y
apt-get install ros-melodic-rplidar-ros -y
sudo apt install python-tornado python-pip ros-melodic-rosbridge-server ros-melodic-web-video-server nginx -y
cd src/
git clone https://github.com/stereolabs/zed-ros-examples.git
git clone https://github.com/stereolabs/zed-ros-interfaces.git
# git clone https://github.com/osrf/rvizweb/
cd ..
