apt install gpsd gpsd-clients python-gps -y
apt-get install ros-melodic-nmea-navsat-driver -y   
apt-get install ros-melodic-robot-localization -y
apt-get install ros-melodic-mapviz -y
apt install ros-melodic-mapviz-plugins -y
apt install ros-melodic-tile-map -y
apt-get install ros-melodic-rplidar-ros -y
sudo apt install python-tornado python-pip ros-melodic-rosbridge-server ros-melodic-web-video-server nginx -y
cd src/
git clone https://github.com/stereolabs/zed-ros-wrapper.git
git clone https://github.com/stereolabs/zed-ros-interfaces.git
# git clone https://github.com/osrf/rvizweb/
git clone https://github.com/CMPE195-Group-28-Auto-Delivery-Robot/garmin18x_usb.git
cd garmin18x_usb
git clone https://github.com/quentinsf/pygarmin.git
cd ..
cd ..
