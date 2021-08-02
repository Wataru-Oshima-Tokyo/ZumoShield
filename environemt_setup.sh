echo “install rosserial”
sudo apt update
sudo apt install -y arduino arduino-core
cd catkin_ws/src
git clone https://github.com/ros-drivers/rosserial.git
cd ../
catkin build
source devel/setup.bash
rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries/
sudo usermod -a -G dialout pi
sudo nano ~/.bashrc

sudo chmod 666 /dev/ttyACM0
cd /etc/udev/rules.d
sudo touch ttyACM0rule.rules
echo ‘KERNEL=="ttyACM0", MODE="0666"’ >> ttyACM0rule.rules