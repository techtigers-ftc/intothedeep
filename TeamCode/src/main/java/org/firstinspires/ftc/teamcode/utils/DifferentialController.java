package org.firstinspires.ftc.teamcode.utils;


import com.qualcomm.robotcore.util.Range;

/**
 * A Controller for calculating the servo positions of a differential claw based on target angles.
 */
public class DifferentialController {
    private final double gearRatio;
    private final double maxServoAngle;

    /**
    * Creates a new Differential Controller
    *
    * @param gearRatio ratio between follower gear and driver gear as driver gear / follower gear
    * @param maxServoAngle the maximum range of both servos
    */
    public DifferentialController(double gearRatio, double maxServoAngle) {
        this.gearRatio = gearRatio;
        this.maxServoAngle = maxServoAngle;
    }

    /**
     * Generates the servo positions for the two servos on the differential claw based on the pitch
     * and rotation angles
     *
     * @param pitchAngle the target pitch for the differential claw
     * @param rotationAngle the target rotation for the differential claw
     * @return positions of the servos, first is left servo second is the right servo
     */
    public double[] GeneratePositions(double pitchAngle, double rotationAngle){
        double[] positions = new double[2];
        positions[0] = pitchAngle - rotationAngle / gearRatio;
        positions[1] = pitchAngle + rotationAngle / gearRatio;

        //Normalize both positions so they are between 0 and 1;
        positions[0] /= maxServoAngle;
        positions[1] /= maxServoAngle;

        positions[0] = Range.clip(positions[0], 0, 1);
        positions[1] = Range.clip(positions[1], 0, 1);

        return positions;
    }
}