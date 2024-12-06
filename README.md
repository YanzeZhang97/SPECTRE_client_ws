**This is the robot/client side implementation of the SPECTRE for ITSC 6166/8166 Computer Communivation and Networks.**

**This is another implementation for the [original project](https://github.com/ibrahim-anas/SPECTRE-ROS2). In this implementation, you are able to run a service in the [workstation side](https://github.com/YanzeZhang97/SPECTRE_server_ws) and run a image collection in the [robot/client side](https://github.com/YanzeZhang97/SPECTRE_client_ws).**

The client side would be a robot or a computer and it will be used for collect the image data in real time. **It require a camera configured on the computer/robot.** The code is tested in the `ROS2 foxy` envrionment. Please make sure you have `ROS2 foxy` installed.

## Running the code

**step 1: Clone the code**

`git clone https://github.com/YanzeZhang97/SPECTRE_client_ws`

**step 2: Make the space**

`colcon build`

**step 3: Run the client**

`ros2 run python_gui pythongui_3`

**step 4: Start Streaming**

You can click the start streaming to stream the images to the network.