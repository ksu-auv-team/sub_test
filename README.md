#Whats up? Give me, Alec, a call/text at 404-580-0061
### This is how you use the current code base:
## 1. Set up ROS
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'; \
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116; \
sudo apt-get update; \
sudo apt-get -y --force-yes install ros-indigo-desktop; \
sudo rosdep init; \
rosdep update; \
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc; \
source ~/.bashrc;

```
## 2. Set up Catkin workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "alias cm='catkin_make -j4 -C ~/catkin_ws/'" >> ~/.bash_aliases
echo "alias refresh='source ~/.bashrc'" >> ~/.bash_aliases
source ~/.bashrc
cm
refresh

```
### 3. Install dependencies:
This is just a decent, untested guess. There may be more dependencies.

    sudo apt-get -y install ros-indigo-mavros ros-indigo-mavros-msgs ros-indigo-mavros-extras ros-indigo-rosserial-arduino ros-indigo-rosserial git

## 5. Install OpenCV
```
cd
git clone https://github.com/Itseez/opencv.git
cd opencv
git checkout 3.1.0
mkdir cv3-release
cd cv3-release
cmake ..
make -j4
sudo make install

```

### 4. Set up arduino (should not be necessary unless arduino fails)
[this link](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)
tells you how to set up arduino IDE for ROS sketches. Following is a simple and probably working method:

    sudo apt-get -y install arduino*
    cd $find catkin_ws 
    catkin_make install
    cd ~/sketchbook/libraries
    rosrun rosserial_arduino make_libraries.py .
    


Now, open and compile the scketch like normal.

If you get 'cannot find arduino on dev/ttyUSB0', run arduino in root mode (sudo arduino)

If you try to communicate with arduino and you get permission errors, try

    sudo chmod 666 /dev/ttyACM0(or whatever the arduino is)
    
    
### 4. Run a single launch file
we are still working on it, but this is the gist of it.


P.S.These are the software files for the 2016 AUV team. 

