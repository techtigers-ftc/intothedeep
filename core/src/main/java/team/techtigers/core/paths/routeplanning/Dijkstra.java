package team.techtigers.core.paths.routeplanning;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.PriorityQueue;

/**
 * The pathfinding algorithm
 *
 * @param <T> The type of the value associated with Nodes
 */
public class Dijkstra<T extends Node> {
    /**
     * Find a path from any point to another with the least possible cost
     * (in our case, cost will be based on distance travelled and the number of changes in direction)
     *
     * @param start The starting Node
     * @param end   The ending Node
     * @return The path of Nodes
     */
    public ArrayList<T> findPath(T start, T end) {
        PriorityQueue<Node> pq = new PriorityQueue<>((n1, n2) -> (int) (n1.getCost() - n2.getCost()));

        start.setCost(0);
        pq.add(start);
        while (pq.size() != 0) {

            Node current = pq.poll();

            for (Iterator<Node> it = current.traverseConnections(); it.hasNext(); ) {
                pq.add(it.next());
            }
            if (current.equals(end)) {
                break;
            }
        }
        if (end.getSource() == null) {
            return new ArrayList<>();
        }
        return backtrack(new ArrayList<>(), end);
    }

    /**
     * Find the path by going backward from the end
     *
     * @param path    The accumulated path
     * @param current The current Node
     * @return The final path
     */
    private static <T extends Node> ArrayList<T> backtrack(ArrayList<T> path, T current) {
        path.add(0, current);
        if (current.getSource() != null) {
            return backtrack(path, (T) current.getSource());
        } else {
            return path;
        }
    }

}
