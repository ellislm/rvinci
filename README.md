# RVinci
Rvinci is an interface between dvrk and RViz. It creates a stereoscopic render window for the da Vinci viewer,
provides camera control within the RViz environment, and provides functionality for 3D cursors to manipulate
objects within the world.

The 3D cursors utilized are found here: https://github.com/aleeper/interaction_cursor_3d and are required
for RVinci.

Any input source can publish rvinci_input messages to
RVinci to utilize the camera controls and 3D cursors.
