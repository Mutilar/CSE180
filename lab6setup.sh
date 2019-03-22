sudo apt-get update
sudo apt-get upgrade
sudo apt-get install ros-melodic-twist-mux
sudo apt-get install ros-melodic-robot-localization
sudo apt-get install ros-melodic-imu*
sudo apt-get install ros-melodic-nmea*
sudo apt-get install ros-melodic-urdf*
sudo apt-get install ros-melodic-um7
sudo apt-get install ros-melodic-amcl ros-melodic-move-base ros-melodic-map-server ros-melodic-dwa-local-planner
rm -rf ~/CSE180/src/husky
cd ~/CSE180/src
wget http://robosrv.ucmerced.edu/huskynew.tgz
tar zxvf huskynew.tgz
cd ~/CSE180
catkin_make

