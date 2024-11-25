package team.techtigers.core.paths.routeplanning;

import java.util.ArrayList;
import java.util.HashMap;

import team.techtigers.core.paths.ClosestNodeIsTooFarException;
import team.techtigers.core.paths.NodeCannotBeFoundException;
import team.techtigers.core.paths.geometry.Point;


/**
 * A graph representing the field
 */
public class FieldGraph {
    private final HashMap<Point, FieldNode> map;

    /**
     * Construct a FieldGraph
     *
     * @param map The graph
     */
    public FieldGraph(HashMap<Point, FieldNode> map) {
        this.map = map;
    }

    /**
     * Reset the graph (set all costs to infinity and all sources to null)
     */
    public void reset() {
        for (FieldNode node : map.values()) {
            node.reset();
        }
    }

    /**
     * Gets the map
     *
     * @return The map
     */
    public HashMap<Point, FieldNode> getMap() {
        return this.map;
    }

    /**
     * Find the closest node in the map to the point provided
     *
     * @param startPoint The starting point
     * @return The FieldNode closest to the point given
     */
    public FieldNode getClosestNode(Point startPoint, double threshold) throws NodeCannotBeFoundException, ClosestNodeIsTooFarException {
        FieldNode closest = null;
        double minDistance = Double.POSITIVE_INFINITY;

        for (FieldNode node : map.values()) {
            Point point = node.getValue().center;
            double newDist = point.dist(startPoint);
            if (newDist < minDistance) {
                minDistance = newDist;
                closest = node;
            }
        }
        if (closest == null) {
            throw new NodeCannotBeFoundException();
        }
        if (minDistance > threshold) {
            throw new ClosestNodeIsTooFarException();
        }
        return closest;
    }

    /**
     * Print out the map for debugging purposes
     */
    public void dump() {
        ArrayList<FieldNode> values = new ArrayList<>(map.values());
        values.sort((FieldNode node1, FieldNode node2) -> {
            double compareX = node1.getValue().center.x - node2.getValue().center.x;
            if (compareX != 0) {
                return (int) compareX;
            } else {
                return (int) (node1.getValue().center.y - node2.getValue().center.y);
            }
        });
        for (FieldNode val : values) {
            val.dump();
        }
    }
}
