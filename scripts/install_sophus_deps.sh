cd /tmp
git clone https://github.com/fmtlib/fmt.git
cd fmt
git checkout 5.3.0
mkdir build
cd build
cmake -DCMAKE_CXX_COMPILER_LAUNCHER=ccache ..
make -j8
make install
cd ../..

apt-get -qq update
apt-get install gfortran libc++-dev libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev libceres-dev
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.4/eigen-3.3.4.tar.bz2
tar xvf eigen-3.3.4.tar.bz2
mkdir build-eigen
cd build-eigen
cmake ../eigen-3.3.4 -DEIGEN_DEFAULT_TO_ROW_MAJOR=$ROW_MAJOR_DEFAULT
make install
cd ..

git clone https://ceres-solver.googlesource.com/ceres-solver ceres-solver
cd ceres-solver
git reset --hard 399cda773035d99eaf1f4a129a666b3c4df9d1b1
mkdir build
cd build
ccache -s
cmake -DCMAKE_CXX_COMPILER_LAUNCHER=ccache ..
make -j8
make install
cd ../..
