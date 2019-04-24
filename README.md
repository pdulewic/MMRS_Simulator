## MMRS Simulator

This simulator models the behavior of a group of robots (MMRS - Multiple Mobile Robot System) that carry out their tasks, or missions, in a common 2D workspace. The workspace is shared by the robots that are realizing their missions, while idling robots stay in their private area, e.g., in a docking
station. For simplicity, these robots are represented as disks with radius *r*. It is assumed that when assigned a mission, a robot also receives, or plans for itself, a path to follow, starting and ending in its private place.

To avoid collisions, we divide paths into sectors and then determine the conflict relations between each pair of sectors. Two sectors are in conflict relation if the distance between them is less than 2*r*. In this simulator, we assume that the path with already determined conflict relations will be given as the input data. Therefore, we can abandon information about their geometry, remembering only their length.

TODO
- describe special poins and events
- add some graphics


