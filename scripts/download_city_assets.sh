#!/bin/bash
# Script to prepare environment for AWS RoboMaker Small City assets and Gazebo Fuel models
echo "Setting up resources for the high-fidelity city world..."

PROJECT_DIR=$(pwd)
MODELS_DIR="$PROJECT_DIR/models"

# Ensure models directory exists
mkdir -p "$MODELS_DIR"

# Export the Gazebo Sim resource path to include our models directory
echo "export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:$MODELS_DIR" >> ~/.bashrc
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$MODELS_DIR

echo "Added $MODELS_DIR to GZ_SIM_RESOURCE_PATH in ~/.bashrc"
echo "Gazebo will automatically download the required Fuel models upon first launch."
echo "Setup complete. You can now launch the city_world.sdf"
