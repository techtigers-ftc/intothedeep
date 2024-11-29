package org.firstinspires.ftc.teamcode.utils;


import com.qualcomm.robotcore.util.Range;

/**
 * A Controller for calculating the servo positions of a differential claw based on target angles.
 */
public class DifferentialController {
    private final double gearRatio;
    private final double maxServoAngle;

    private final double maxDriverRange;

    private final double servoGearRatio;

    private double maxPitchAngle;
    private double maxRotationAngle;

    /**
     * Creates a new Differential Controller
     *
     * @param gearRatio      ratio between follower gear and driver gear as driver gear / follower gear
     * @param maxServoAngle  the maximum range of both servos
     * @param servoGearRatio the ration between the gear on the servo and the driver gear of the
     *                       differential claw (servo gear / driver gear)
     */
    public DifferentialController(double gearRatio, double maxServoAngle, double servoGearRatio) {
        this.gearRatio = gearRatio;
        this.maxServoAngle = maxServoAngle;
        this.servoGearRatio = servoGearRatio;

        maxDriverRange = servoGearRatio * maxServoAngle;
        maxPitchAngle = maxDriverRange / 2.0;
        maxRotationAngle = maxDriverRange / 2.0;
    }

    /**
     * Sets the maximum range of the pitch and rotation
     *
     * @param maxPitchAngle    the maximum pitch angle in degrees
     * @param maxRotationAngle the maximum rotation angle in degrees
     */
    public void setMaxRange(double maxPitchAngle, double maxRotationAngle) {
        if (maxPitchAngle + maxRotationAngle > maxDriverRange) {
            throw new IllegalArgumentException("The sum of the maxPitchAngle and maxRotationAngle must be less than or equal to the maxClawAngle");
        }
        this.maxPitchAngle = maxPitchAngle;
        this.maxRotationAngle = maxRotationAngle;
    }

    /**
     * Sets the maximum pitch angle lowering the maximum rotation angle if necessary
     *
     * @param maxPitchAngle the maximum pitch angle in degrees
     */
    public void setMaxPitch(double maxPitchAngle) {
        this.maxPitchAngle = maxPitchAngle;
        if (maxPitchAngle + maxRotationAngle > maxDriverRange) {
            maxRotationAngle = maxDriverRange - maxPitchAngle;
        }
    }

    /**
     * Sets the maximum rotation angle lowering the maximum pitch angle if necessary
     *
     * @param maxRotationAngle the maximum rotation angle in degrees
     */
    public void setMaxRotation(double maxRotationAngle) {
        this.maxRotationAngle = maxRotationAngle;
        if (maxPitchAngle + maxRotationAngle > maxDriverRange) {
            maxPitchAngle = maxDriverRange - maxRotationAngle;
        }
    }

    public double getMaxPitchAngle() {
        return maxPitchAngle;
    }

    public double getMaxRotationAngle() {
        return maxRotationAngle;
    }

    /**
     * Generates the servo positions for the two servos on the differential claw based on the pitch
     * and rotation angles
     * From the perspective of the robot down pitch and left rotation are positive
     *
     * @param pitchAngleDegrees    the target pitch for the differential claw in degrees from 0 to
     *                             max
     * @param rotationAngleDegrees the target rotation for the differential claw in degrees from 0
     *                             to max
     * @return positions of the servos, first is left servo second is the right servo
     */
    public double[] calculateServoPositions(double pitchAngleDegrees, double rotationAngleDegrees) {
        pitchAngleDegrees = Range.clip(pitchAngleDegrees, 0, maxPitchAngle);
        rotationAngleDegrees = Range.clip(rotationAngleDegrees, 0, maxRotationAngle);

        double leftServoPosition = pitchAngleDegrees - rotationAngleDegrees / gearRatio;
        double rightServoPosition = pitchAngleDegrees + rotationAngleDegrees / gearRatio;

        leftServoPosition /= servoGearRatio;
        rightServoPosition /= servoGearRatio;

        //Normalize both positions so they are between 0 and 1;
        leftServoPosition /= maxServoAngle;
        rightServoPosition /= maxServoAngle;

        leftServoPosition = Range.clip(leftServoPosition, 0.0, 1.0);
        rightServoPosition = Range.clip(rightServoPosition, 0.0, 1.0);

        return new double[]{leftServoPosition, rightServoPosition};
    }

    /**
     * Returns the pitch and rotation angles based on the servo positions
     *
     * @param leftServoPosition  the left servos position from 0 to 1
     * @param rightServoPosition the right servos position from 0 to 1
     * @return the pitch and rotation angles in degrees first is pitch second is rotation
     */
    public double[] getPitchAndRotation(double leftServoPosition, double rightServoPosition) {
        leftServoPosition *= maxServoAngle;
        rightServoPosition *= maxServoAngle;

        double[] positions = new double[2];
        positions[0] = (leftServoPosition + rightServoPosition) / gearRatio;
        positions[1] = (gearRatio * (leftServoPosition - rightServoPosition)) / 2.0;

        positions[0] /= servoGearRatio;
        positions[1] /= servoGearRatio;

        return positions;
    }
}