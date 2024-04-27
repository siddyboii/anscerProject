## Algorithm Overview

### Trajectory Collection

The trajectory collection process involves subscribing to the `/joint_states` topic to receive real-time updates of the robot's position. Each received message contains pose data which is processed and converted into a visualization marker.

Pseudocode for Trajectory Collection:

Subscribe to /joint_states
On receiving a path message:
    For each pose in the path message:
        Create a visualization marker at the pose position
        Add the marker to the MarkerArray
    Publish the MarkerArray to a visualization topic

### Trajectory Storage
Trajectory storage is triggered by a service call, which specifies the filename and format for the data to be saved. The service callback fetches the trajectory data from the current session and writes it to the specified file.

Pseudocode for Trajectory Storage:

Service Callback for save_trajectory:
    Open/Create file with the specified filename
    If the format is CSV:
        Write header to file
        For each pose in the trajectory data:
            Write pose information to file as CSV
    Close the file
    Return success response


### Trajectory Visualization
Trajectory visualization utilizes RViz to display the trajectory of the robot. The MarkerArray published during the collection phase is used to visualize the path.

Pseudocode for Trajectory Visualization:

On startup:
    Load RViz with the pre-configured settings from rviz_config.rviz
During operation:
    Listen for MarkerArray messages on the visualization topic
    Update the visualization in RViz with the received MarkerArray
