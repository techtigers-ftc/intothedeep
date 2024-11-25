package team.techtigers.core.paths;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;

import team.techtigers.core.paths.geometry.Point;
import team.techtigers.core.paths.routeplanning.Dijkstra;
import team.techtigers.core.paths.routeplanning.FieldGraph;
import team.techtigers.core.paths.routeplanning.FieldNode;
import team.techtigers.core.paths.routeplanning.GraphBuilder;


/**
 * Returns a set of waypoints given a starting and ending position, accounting
 * for any known obstacles on the field.
 */
public class Pathfinder {
    private static final double BACK_ANGLE_TOLERANCE = Math.toRadians(45);
    private static Pathfinder instance;
    private final FieldGraph graph;

    private final Dijkstra planner;
    private final double divisionsPerTile;


    /**
     * Constructs Pathfinder
     *
     * @param fieldMap         The map of the field Pathfinder uses to route
     * @param divisionsPerTile The number of subdivisions per tile in one direction
     */
    private Pathfinder(int[][] fieldMap, double divisionsPerTile) {
        graph = GraphBuilder.buildGraph(fieldMap, divisionsPerTile);
        planner = new Dijkstra<>();
        this.divisionsPerTile = divisionsPerTile;
    }

    /**
     * Initialize the Pathfinder instance
     *
     * @param fieldMap         The map of the field Pathfinder uses to route
     * @param divisionsPerTile The number of subdivisions per tile in one direction
     */
    public static void initialize(int[][] fieldMap, double divisionsPerTile) {
        instance = new Pathfinder(fieldMap, divisionsPerTile);
    }

    /**
     * Get the Pathfinder instance, given that it has been initialized
     *
     * @return The Pathfinder instance
     */
    public static Pathfinder getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Pathfinder has not been initialized");
        }
        return instance;
    }

    private static double getAngleBetween3Points(Waypoint first, Waypoint second, Waypoint third) {
        Point firstVector = first.getPoint();
        Point secondVector = second.getPoint();
        Point thirdVector = third.getPoint();

        Point secondToFirst = firstVector.minus(secondVector);
        Point secondToThird = thirdVector.minus(secondVector);

        return secondToFirst.angleTo(secondToThird);
    }

    private boolean pointCloseEnough(Waypoint point1, Waypoint point2) {
        double tolerance = 2.5;
        boolean xEquals = Math.abs(point1.getX() - point2.getX()) < tolerance;
        boolean yEquals = Math.abs(point1.getY() - point2.getY()) < tolerance;
        boolean headingEquals = Math.abs(point1.getHeading() - point2.getHeading()) < tolerance;

        return xEquals && yEquals && headingEquals;
    }

    private ArrayList<Waypoint> pruneNodes(ArrayList<Waypoint> nodes) {
        ArrayList<Waypoint> newList = new ArrayList<>();
        ArrayList<Waypoint> queue = new ArrayList<>();

        for (int index = 0; index < nodes.size(); index++) {
            if (queue.size() < 3) {
                queue.add(nodes.get(index));
            }
            if (queue.size() == 3) {
                double angle = getAngleBetween3Points(queue.get(0), queue.get(1), queue.get(2));
                double angleDelta = Math.abs(Math.toRadians(180) - angle);
                boolean removeNode = angleDelta < 0.001;
                if (removeNode) {
                    queue.remove(1);
                } else {
                    newList.add(queue.get(0));
                    queue.remove(0);
                }
            }
        }

        newList.addAll(queue);

        return newList;
    }

    /**
     * Overload to generate a path while defaulting pruneNodes to true
     *
     * @param startPoint The starting point
     * @param endPoint   The ending point
     * @return A list of waypoints that represent the path
     */
    public ArrayList<Waypoint> generatePath(Waypoint startPoint, Waypoint endPoint) throws ClosestNodeIsTooFarException, PathCannotBeFoundException, NodeCannotBeFoundException {
        return generatePath(startPoint, endPoint, true);
    }


    /**
     * Generates a set of waypoints from any pose to another
     *
     * @param startPoint The starting point
     * @param endPoint   The ending point
     * @param pruneNodes Whether to remove nodes on the same line or not.
     *                   Ex: (0,0), (1,0), (2,0), prune nodes would remove (1,0)
     *                   because it's on the same line as the other 2 points
     * @return A list of waypoints that represent the path
     */
    public ArrayList<Waypoint> generatePath(Waypoint startPoint,
                                            Waypoint endPoint, boolean pruneNodes) throws ClosestNodeIsTooFarException, NodeCannotBeFoundException, PathCannotBeFoundException {
        if (pointCloseEnough(startPoint, endPoint)) {
            throw new PathCannotBeFoundException();
        }

        graph.reset();

        double threshold = 36 / this.divisionsPerTile * Math.sqrt(2);

        FieldNode start = graph.getClosestNode(startPoint.getPoint(), threshold);
        FieldNode end = graph.getClosestNode(endPoint.getPoint(), threshold);

        if (start.getValue().center.equals(end.getValue().center)
                || startPoint.getPoint().dist(endPoint.getPoint())
                <= startPoint.getPoint().dist(start.getValue().center)) {
            return new ArrayList<>(
                    Arrays.asList(startPoint, endPoint));
        }

        ArrayList<FieldNode> nodePath = planner.findPath(start, end);

        ArrayList<Waypoint> path = (ArrayList<Waypoint>)
                nodePath.stream().map((fieldNode -> new Waypoint(fieldNode.getValue().center)))
                        .collect(Collectors.toList());

        double headingDiff = (endPoint.getHeading() - startPoint.getHeading()) / (path.size() - 1);
        for (int index = 0; index < path.size(); index++) {
            Waypoint currentWaypoint = path.get(index);
            path.set(index, new Waypoint(currentWaypoint.getX(), currentWaypoint.getY(),
                    startPoint.getHeading() + (index * headingDiff)));
        }

        if (path.isEmpty()) {
            throw new PathCannotBeFoundException();
        }

        // If start pose and start node center pose is the same, do not add the start pose point
        if (!pointCloseEnough(startPoint, path.get(0))) {
            path.add(0, startPoint);
        }

        // If end pose and end node center pose is the same, do not add the end pose point
        if (!pointCloseEnough(endPoint, path.get(path.size() - 1))) {
            path.add(endPoint);
        }

        // Removing back and forth movement that stalls calculation in beginning
        if (path.size() > 2) {
            double startAngle = getAngleBetween3Points(path.get(0),
                    path.get(1), path.get(2));
            if (Math.abs(startAngle) < BACK_ANGLE_TOLERANCE) {
                path.remove(1);
            }
        }

        // Removing back and forth movement that stalls calculation at end
        if (path.size() > 2) {
            double endAngle = getAngleBetween3Points(path.get(path.size() - 3),
                    path.get(path.size() - 2),
                    path.get(path.size() - 1));
            if (Math.abs(endAngle) < BACK_ANGLE_TOLERANCE) {
                path.remove(path.size() - 2);
            }
        }

        if (pruneNodes) {
            path = pruneNodes(path);
        }

        return path;
    }
}
