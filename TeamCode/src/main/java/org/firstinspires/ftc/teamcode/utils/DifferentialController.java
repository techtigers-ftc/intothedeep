package org.firstinspires.ftc.teamcode.utils;


import com.qualcomm.robotcore.util.Range;

/**
 * A Controller for calculating the servo positions of a differential claw based on target angles.
 */
public class DifferentialController {
    private final double gearRatio;
    private final double maxServoAngle;

    private final double maxClawAngle;

    private final double servoGearRatio;

    private double maxPitchAngle;
    private double maxRotationAngle;

    /**
     * Creates a new Differential Controller
     *
     * @param gearRatio ratio between follower gear and driver gear as driver gear / follower gear
     * @param maxServoAngle the maximum range of both servos
     */
    public DifferentialController(double gearRatio, double maxServoAngle, double servoGearRatio) {
        this.gearRatio = gearRatio;
        this.maxServoAngle = maxServoAngle;
        maxClawAngle = servoGearRatio * maxServoAngle;
        maxPitchAngle = maxClawAngle / 2;
        maxRotationAngle = maxClawAngle / 2;
        this.servoGearRatio = servoGearRatio;
    }

    /**
     * Sets the maximum range of the pitch and rotation
     * @param maxPitchAngle the maximum pitch angle in degrees
     * @param maxRotationAngle the maximum rotation angle in degrees
     */
    public void maxRange(double maxPitchAngle, double maxRotationAngle){
        if(maxPitchAngle + maxRotationAngle > maxClawAngle){
            throw new IllegalArgumentException("The sum of the maxPitchAngle and maxRotationAngle must be less than or equal to the maxClawAngle");
        }
        this.maxPitchAngle = maxPitchAngle;
        this.maxRotationAngle = maxRotationAngle;
    }

    /**
     * Sets the maximum pitch angle lowering the maximum rotation angle if necessary
     * @param maxPitchAngle the maximum pitch angle in degrees
     */
    public void maxPitch(double maxPitchAngle){
        this.maxPitchAngle = maxPitchAngle;
        if(maxPitchAngle + maxRotationAngle > maxClawAngle){
            maxRotationAngle = maxClawAngle - maxPitchAngle;
        }
    }
    /**
     * Sets the maximum rotation angle lowering the maximum pitch angle if necessary
     * @param maxRotationAngle the maximum rotation angle in degrees
     */
    public void maxRotation(double maxRotationAngle){
        this.maxRotationAngle = maxRotationAngle;
        if(maxPitchAngle + maxRotationAngle > maxClawAngle){
            maxPitchAngle = maxClawAngle - maxRotationAngle;
        }
    }

    /**
     * Generates the servo positions for the two servos on the differential claw based on the pitch
     * and rotation angles
     *
     * @param pitchAngle the target pitch for the differential claw
     * @param rotationAngle the target rotation for the differential claw
     * @return positions of the servos, first is left servo second is the right servo
     */
    public double[] generatePositions(double pitchAngle, double rotationAngle){
        if(pitchAngle > maxPitchAngle || rotationAngle > maxRotationAngle){
            throw new IllegalArgumentException("The pitch and rotation angles must be less than the max");
        }

        double[] positions = new double[2];
        positions[0] = pitchAngle - rotationAngle / gearRatio;
        positions[1] = pitchAngle + rotationAngle / gearRatio;

        positions[0] /= servoGearRatio;
        positions[1] /= servoGearRatio;

        //Normalize both positions so they are between 0 and 1;
        positions[0] /= maxServoAngle;
        positions[1] /= maxServoAngle;

        positions[0] = Range.clip(positions[0], 0, 1);
        positions[1] = Range.clip(positions[1], 0, 1);

        return positions;
    }

    /** Returns the pitch and rotation angles based on the servo positions
     *
     * @param leftServoPosition the left servos position from 0 to 1
     * @param rightServoPosition the right servos position from 0 to 1
     * @return the pitch and rotation angles in degrees first is pitch second is rotation
     */
    public double[] getPitchAndRotation(double leftServoPosition, double rightServoPosition){
        leftServoPosition *= maxServoAngle;
        rightServoPosition *= maxServoAngle;

        double[] positions = new double[2];
        positions [0] = (leftServoPosition+rightServoPosition) / gearRatio;
        positions [1] = (gearRatio * (leftServoPosition-rightServoPosition)) / 2;

        positions[0] /= servoGearRatio;
        positions[1] /= servoGearRatio;

        return  positions;
    }
}