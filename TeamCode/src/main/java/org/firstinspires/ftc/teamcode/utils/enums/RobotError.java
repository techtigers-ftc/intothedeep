package org.firstinspires.ftc.teamcode.utils.enums;

/**
 * Enum for all the errors that the robot can encounter
 */
public enum RobotError {
    /**
     * No errors
     */
    NO_ERROR(0),

    /**
     * Attempting to transition to an intake position from an invalid start position
     */
    INVALID_INTAKE_POSITION(1),

    /**
     * Attempting to transition to a dropper position from an invalid start position
     */
    INVALID_DROPPER_POSITION(2);

    /**
     * The error code associated with the error
     */
    public final int code;

    /**
     * Constructor for the RobotError enum
     *
     * @param code The error code
     */
    RobotError(int code) {
        this.code = code;
    }
}
