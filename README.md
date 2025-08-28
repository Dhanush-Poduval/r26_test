<h1>Rudra Test</h1>
<br>

The project consists of two main components:
<br>
->Planner – responsible for path planning on a grid.
->Odometry – responsible for computing time and rotation commands along the planned path.
<br>

<h1>Thought Process</h1>

I started by understanding the grid map representation and how obstacles are handled.

The path planning uses a variant of the A* algorithm with an 8-connected grid and Chebyshev distance as the method to calculate the distance without actually going to each path itself

For odometry, I needed to compute distances and angles between consecutive points that could be accessed, while accumulating total time and total rotation angle.

<h1>Challenges faced</h1>

Angle Calculation: Computing the total turning angle was tricky, especially when consecutive moves change direction. Normalizing the angle to the [0,360]range and accumulating turns correctly took several iterations.
Plus the a* method is very efficient for time taken but was not able to showcase the exact angle result as it was going to axis-line points , while a* method was accessing diagonal elements also.

PathPlanner: First the bfs was leading to net time taken around 21s which was very hard to understand why but now after a* i am able to showcase the required timetaken as the output

Debugging: Linking errors in C++ for the pathplanner part slowed down development. Ensuring that all methods were correctly defined in the .cpp files took time . Also just compiling and running the files without the make commands took time to understand and debug .
<br>

<h1>Resources used</h1>
->Mainly used chatgpt for the bfs to a* switch as well as debugging a*code  , when it came to the function that calculated the distance (hueristic) i also used the assistance of chatgpt and geeksforgeeks to understand it a bit better and for the grid creation i used chatgpt to understand what was happening and what the input format was 
->For the odometric functions the angle normalization debugging i used chatgpt and stackoverflow but couldnt find the issue leading to wrong answer for the degree

<h1>Flow of the project</h1>
The main function calls the grid creating fucntion from gridmap.cpp and that converts the gps coordinates into a grid type layout than the ubslayout gives us the start point and the goal end point the pathfinder finds and creates the best path between the start endpoint and the goal and the odometry.cpp file calculates the time taken and the angles rotated by the bot about each point of the path 

<h1>Implementation</h1>

Path Planning (planning.cpp):

Used A* search to find the best path from start to goal on a grid.

Allowed diagonal moves too (8-connected grid).

Used Chebyshev distance (max of row/column differences) as a guide to reach the goal faster.

Kept track of the cost to reach each cell and the parent of each cell to reconstruct the path.

Odometry (odometry.cpp):

Calculates distance between each path point to find time taken using wheel radius and RPM.

Calculates angle changes between path segments to find total rotation of the bot.

Normalizes angles so turns are counted correctly.

Testing:

Printed the grid and path to check visually.

Compared odometry outputs (time and angle) with expected results.

Debugged angle and distance calculations to get outputs closer to expected.



