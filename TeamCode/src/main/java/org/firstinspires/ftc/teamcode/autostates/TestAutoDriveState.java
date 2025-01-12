package org.firstinspires.ftc.teamcode.autostates;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A class for testing autonomous drives
 */
public class TestAutoDriveState extends DriveStateBase{

    /**
     * Constructor for the TestAutoDriveState
     *
     * @param name The name of the state
     * @param drive The drive subsystem
     * @param robotState The robot state
     */
    public TestAutoDriveState(String name, DriveSubsystem drive, RobotState robotState) {
        super(name, drive, robotState);
        addCommands(
                autoDriveCommand
        );
    }
}
