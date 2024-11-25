package team.techtigers.core.paths.routeplanning;


import java.util.HashMap;

import team.techtigers.core.paths.geometry.Point;
import team.techtigers.core.paths.geometry.Rectangle;

/**
 * The class used to create graphs representing the game field
 */
public class GraphBuilder {

    /**
     * Builds a graph using the 36 tiles on the field, the provided map, and the number of divisions per tile.
     * A standard field goes from -72 to 72 in 2 axes
     *
     * @param fieldMap         A square 2D array with only 1s and 0s, with each value representing whether or not the
     *                         robot is allowed to travel to that coordinate. Use this field to block out known obstacles.
     * @param divisionsPerTile Used to scale the coordinates of the final graph. The number of
     *                         divisions in one direction per each field tile
     *                         used.
     * @return A graph representing the field given
     */
    public static FieldGraph buildGraph(int[][] fieldMap, double divisionsPerTile) {

        HashMap<Point, FieldNode> map = new HashMap<>();
        for (int x = 0; x < fieldMap.length; x++) {
            for (int y = 0; y < fieldMap[x].length; y++) {
                int xIndex = -1 - x + fieldMap.length / 2;
                int yIndex = y - fieldMap[x].length / 2;

                // Doing this because x and y values on graph are flipped to
                // what they're supposed to be
                Point bottomLeft = new Point(yIndex, xIndex);
                boolean isIncluded = fieldMap[x][y] != 0;
                FieldNode node = new FieldNode(new Rectangle(
                        new Point(bottomLeft.x + 1, bottomLeft.y + 1),
                        bottomLeft
                ));
                if (isIncluded) {
                    map.put(bottomLeft, node);
                }
            }
        }

        for (Point point : map.keySet()) {
            FieldNode current = map.get(point);
            for (int dx = -1; dx < 2; dx++) {
                for (int dy = -1; dy < 2; dy++) {
                    Point neighbor = point.add(new Point(dx, dy));
                    if (map.containsKey(neighbor)) {
                        map.get(neighbor).addConnection(new Connection(current, point.dist(neighbor)));
                    }
                }
            }
        }

        for (Point k : map.keySet()) {
            FieldNode n = map.get(k);
            // This will not work for non-square tiles.
            n.getValue().scale(24.0 / divisionsPerTile);
        }


        return new FieldGraph(map);
    }
}
