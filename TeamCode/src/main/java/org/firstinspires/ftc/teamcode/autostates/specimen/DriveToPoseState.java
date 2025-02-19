package org.firstinspires.ftc.teamcode.autostates.specimen;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A state to drive to a general location
 */
public class DriveToPoseState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToPoseState.class.getSimpleName();

    /**
     * Constructor for a DriveToPose state
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param robotState The robot state
     * @param timeout    The timeout for the state
     */
    public DriveToPoseState(String name, DriveSubsystem drive, RobotState robotState, double timeout) {
        super(name, drive, robotState, timeout);
        addCommands(
                autoDriveCommand
        );
    }
}
