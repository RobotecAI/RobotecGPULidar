
BUILD_DIR=$1
E2E_DIR=$2
INSTALL_DIR=${E2E_DIR}/Assets/E2ESimulator/Sensors/RobotecGPULidar/SimpleLidarAsset/Plugins

echo "Installing from ${BUILD_DIR} to ${E2E_DIR}" &&
cp ${BUILD_DIR}/libRobotecGPULidar.so ${INSTALL_DIR}/Linux/x86_64/ &&
echo "Install done"

