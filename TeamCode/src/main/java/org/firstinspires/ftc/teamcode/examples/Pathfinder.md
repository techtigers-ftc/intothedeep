## Pathfinder
Pathfinder is a tool used to generate optimal paths that avoid known obstacles on the playing field (like the trusses in Centerstage). Its main use case is to improve the robustness of the autonomous period by regenerating paths following collisions or the robot veering off course.

Here's how to use pathfinder...
1. Create a binary 2-dimensional array that represents the field.  If a spot is a 0, the robot is not allowed to go there. If a spot is a 1, it is. Feel free to use our tool (https://docs.google.com/spreadsheets/d/1fQnqgvYEBIYwegxXyzbRSpz-gvW6uzUYO_FL9lWaHa8/edit?usp=sharing) to speed up your graph-generation process.
   `private int[][] fieldMap = ...;`
2. Decide on the number of divisions per field tile: the number of sub-regions which can fit on the length (or width) of a field tile. Why do we ask for this if we can just compute it from the dimensions of `fieldMap`?  It's because while standard FTC tournaments use square fields, competitions like CRI (Chicago Robotics Invitational) do not, and we want to provide that flexibility. Traditionally, you should just use
   `private divisionsPerTile = fieldMap.length / 6;`
3. Initialize pathfinder and get an instance.
   `Pathfinder.initialize(fieldMap, divisionsPerTile);`
   `Pathfinder pf = Pathfinder.getInstance();`
4. Decide on a start and end pose `(x, y, θ)`. `θ` is in degrees, and `x` and `y` are in inches.
   `start = Waypoint(Point(1, 1), 0)`
5. `start = Waypoint(Point(10, 10), 180)`
5. Run Pathfinder
   `ArrayList<Waypoint> path = pf.generatePath(start, end);`

... and you're done!