#Build plugin
cd plugin
mkdir build
cd build
cmake ..
make

#Copy models
cd ../..
mkdir ~/.gazebo/models
cp -r models/* ~/.gazebo/models/
