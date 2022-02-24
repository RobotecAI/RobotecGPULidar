#!/bin/bash 

function wait_for_decision {
	read -p "Continue [type n if not]?" yn
	case $yn in
		[Nn]*) exit 0;;
		* ) echo "Moving on";;
	esac
}

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# ============= USER SPECIFIC =========================================================
e2e_dir=/home/pzyskowski/repos/E2ESimulator
cmake_dir=/home/pzyskowski/tools/cmake-3.22.2-linux-x86_64/bin
fmt_install_dir=/home/pzyskowski/tools/fmt-8.0.1/_build/install/usr/local/lib/cmake/fmt
optix_dir=/home/pzyskowski/tools/NVIDIA-OptiX-SDK-7.2.0-linux64-x86_64
# =====================================================================================

cuda_dir=/usr/local/cuda-11.6/

repo_dir=$SCRIPT_DIR
build_dir=$repo_dir/_build
install_dir=$repo_dir/_install

mkdir $build_dir 
cd $build_dir

export OptiX_INSTALL_DIR=$optix_dir

export PATH=${cmake_dir}:${cuda_dir}/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=$cuda_dir/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
export fmt_DIR=$fmt_install_dir

echo $PATH
echo $LD_LIBRARY_PATH
echo $OptiX_INSTALL_DIR
echo $CMAKE_MODULE_PATH
echo $FMT_DIR
echo $cmake_dir

echo "Found cmake version: $(cmake --version)"

echo "PATH=${cuda_dir}:$PATH LD_LIBRARY_PATH=$cuda_dir -DOptiX_INSTALL_DIR=$optix_dir fmt_DIR=$fmt_install_dir" 

cmake $repo_dir -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DBUILD_TESTS=1
make
#make DESTDIR=$install_dir install

wait_for_decision

e2e_plugin_dir=$e2e_dir/Assets/E2ESimulator/Sensors/RobotecGPULidar/SimpleLidarAsset/Plugins
e2e_linux_plugin_dir=$e2e_plugin_dir/Linux/x86_64/

cp $build_dir/libRobotecGPULidar.so $e2e_linux_plugin_dir
cp $build_dir/dotnet_plugin/gpu-lidar-raycaster-dotnet/gpu_lidar_raycaster_dotnet.dll $e2e_plugin_dir

