cd /tmp
git clone https://github.com/strasdat/Sophus.git 
cd Sophus
mkdir build && cd build
cmake ..
make -j4
make install
