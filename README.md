# tsid_talos_dart

A talos simulation with robot_dart using a tsid stack of tasks

# Install

Create an installation directory
```
cd ~
mkdir install
```

## yaml-cpp

```
sudo apt-get install libyaml-cpp-dev
```

## pinocchio

Install pinnochio following [this](https://github.com/stack-of-tasks/pinocchio)

Use master branch and :
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX='~/install' -DCMAKE_PREFIX_PATH='~/install' ..
make 
sudo make install
```
## tsid

Install tsid following [this](https://github.com/stack-of-tasks/tsid)

Use master branch and :
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX='~/install' -DCMAKE_PREFIX_PATH='~/install' ..
make 
sudo make install
```

## robot_dart

Install tsid following [this](https://github.com/stack-of-tasks/tsid)
Install dart in ~/install
Use master branch and :
```
./waf configure --dart=~/install --prefix=~/install
./waf
./waf install
```

## tsid_robot_dart
```
cd tsid_robot_dart
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX='~/install' -DCMAKE_PREFIX_PATH='~/install' ..
make 
sudo make install
```