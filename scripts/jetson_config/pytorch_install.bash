#
# install prerequisites (many of these are for numpy)
#
sudo apt-get -y update
sudo apt-get -y install autoconf bc build-essential g++-8 gcc-8 clang-8 lld-8 \
	gettext-base gfortran-8 iputils-ping libbz2-dev libc++-dev libcgal-dev \
	libffi-dev libfreetype6-dev libhdf5-dev libjpeg-dev liblzma-dev libncurses5-dev \
	libncursesw5-dev libpng-dev libreadline-dev libssl-dev libsqlite3-dev libxml2-dev \
	libxslt-dev locales moreutils openssl python-openssl rsync scons python3-pip libopenblas-dev

export TORCH_INSTALL=https://developer.download.nvidia.cn/compute/redist/jp/v50/pytorch/torch-1.12.0a0+84d1cb9.nv22.4-cp38-cp38-linux_aarch64.whl

pip install --upgrade pip
pip install expecttest xmlrunner hypothesis aiohttp numpy=='1.19.4' pyyaml scipy=='1.5.3' ninja cython typing_extensions protobuf
export "LD_LIBRARY_PATH=/usr/lib/llvm-8/lib:$LD_LIBRARY_PATH"
pip install --upgrade protobuf
pip install --no-cache $TORCH_INSTALL

#
# torchvision 0.4
#
export TORCHVISION_VERSION=v0.13.0
export TORCH_CUDA_ARCH_LIST="5.3;6.2;7.2;8.7"

echo "torchvision version = $TORCHVISION_VERSION"
echo "TORCH_CUDA_ARCH_LIST = $TORCH_CUDA_ARCH_LIST"

sudo apt install -y --no-install-recommends \
	git \
	build-essential \
	libjpeg-dev \
	zlib1g-dev 

git clone -b ${TORCHVISION_VERSION} https://github.com/pytorch/vision torchvision
cd torchvision 
python3 setup.py install
cd ../
# rm -rf torchvision

# patch for https://github.com/pytorch/pytorch/issues/45323
export PYTHON_ROOT=`pip show torch | grep Location: | cut -d' ' -f2`
export TORCH_CMAKE_CONFIG=$PYTHON_ROOT/torch/share/cmake/Torch/TorchConfig.cmake
echo "patching _GLIBCXX_USE_CXX11_ABI in ${TORCH_CMAKE_CONFIG}"
sed -i 's/  set(TORCH_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=")/  set(TORCH_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=0")/g' ${TORCH_CMAKE_CONFIG}


#
# PyCUDA
#
export PATH="/usr/local/cuda/bin:${PATH}"
export LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"
# echo "$PATH" && echo "$LD_LIBRARY_PATH"

# pip install --no-cache-dir --verbose pycuda six