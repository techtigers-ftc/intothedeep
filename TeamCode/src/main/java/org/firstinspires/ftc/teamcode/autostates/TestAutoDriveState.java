package org.firstinspires.ftc.teamcode.autostates;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A class for testing autonomous drives
 */
public class TestAutoDriveState extends DriveStateBase{

    public TestAutoDriveState(String name, DriveSubsystem drive, RobotState robotState) {
        super(name, drive, robotState);
        addCommands(
                autoDriveCommand
        );
    }
}
