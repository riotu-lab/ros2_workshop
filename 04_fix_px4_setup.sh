# Step 1: Clean the repository and submodules
cd ~/PX4-Autopilot
echo "Cleaning repository and submodules..."
make submodulesclean
make clean
make distclean

# Step 2: Checkout to specific version
echo "Checking out to version v1.14.0..."
git checkout v1.14.0

# Step 3: Clean the repository and submodules again
echo "Performing a second round of cleaning after checkout..."
make submodulesclean
make clean
make distclean

echo "Repository is now at version v1.14.0 and cleaned."

cd ~/PX4-Autopilot
git submodule update --init --recursive
make px4_sitl jmavsim

