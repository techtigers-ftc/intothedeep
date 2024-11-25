package org.firstinspires.ftc.teamcode.examples.pathfinder;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

import team.techtigers.base.BaseOpMode;
import team.techtigers.core.paths.ClosestNodeIsTooFarException;
import team.techtigers.core.paths.NodeCannotBeFoundException;
import team.techtigers.core.paths.PathCannotBeFoundException;
import team.techtigers.core.paths.Pathfinder;
import team.techtigers.core.paths.Waypoint;

/**
 * Test op mode for pathfinder
 */
@TeleOp
public class PathfinderTestOpMode extends BaseOpMode {
    @Override
    public void initialize() {
        // Initialize Pathfinder. The second parameter is the divisions per
        // tile, which is the length of the graph divided by the number of
        // tiles on the field.
        Pathfinder.initialize(Graph.MAP, Graph.MAP.length / 6.0);

        // Attempt to generate a path from the bottom left to the top left
        Waypoint start = new Waypoint(-72, -72);
        Waypoint end = new Waypoint(-72, 72);

        ArrayList<Waypoint> waypoints;
        try {
            // NOTE: This will find a path while pruning any unnecessary nodes.
            //       To see a path with all nodes, add a false argument to the
            //       method call
            waypoints = Pathfinder.getInstance().generatePath(start, end);
            for (Waypoint waypoint : waypoints) {
                telemetry.addLine(waypoint.toString());
            }
        } catch (ClosestNodeIsTooFarException e) {
            throw new RuntimeException(e);
        } catch (NodeCannotBeFoundException e) {
            throw new RuntimeException(e);
        } catch (PathCannotBeFoundException e) {
            throw new RuntimeException(e);
        }

        telemetry.update();
    }
}
