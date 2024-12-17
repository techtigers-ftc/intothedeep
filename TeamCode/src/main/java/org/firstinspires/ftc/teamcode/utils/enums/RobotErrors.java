package org.firstinspires.ftc.teamcode.utils.enums;

/**
 * A class that stores any errors that might be occuring inside the robot
 */
public enum RobotErrors {
    //TODO: Add more errors as they are found
    //TODO: Implement the setting of these errors
    BLOCK_NOT_FOUND(1),
    INTAKE_EXTENSION_TOO_FAR(2);

    public final int code;

    RobotErrors(int code) {
        this.code = code;
    }
}
