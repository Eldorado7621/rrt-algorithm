# RRT-Algorithm
<hr>
The Rapid expanding Random Tree is a path planning algorithm that uses random smaples from the search space too construct a tree. The root of the tree is the starting point. Each node has only one parent.  A connection is made between each sample and the closest state in the tree as it is being drawn. The new state is added to the tree if the link is viable (passes totally through empty space and adheres to any limitations). In this project, the RRT algorithm was implemented in MATLAB with GUI for visualization of the tree growth
