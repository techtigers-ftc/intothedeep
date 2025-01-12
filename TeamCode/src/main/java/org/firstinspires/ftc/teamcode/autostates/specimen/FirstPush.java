package org.firstinspires.ftc.teamcode.autostates.specimen;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

/**
 * A State to drive to the submersible position
 */
public class FirstPush extends DriveStateBase {
    private static final String LOG_TAG =
            FirstPush.class.getSimpleName();
    private static final double TOLERANCE = 1.5;
    private static final double ANGULAR_TOLERANCE = Math.toRadians(5);

    /**
     * Constructor for the DriveToSubmersible
     *
     * @param name The name of the state
     * @param drive The drive subsystem
     * @param robotState The robot state
     */
    public FirstPush(String name, DriveSubsystem drive, RobotState robotState) {
        super(name, drive, robotState);
        addCommands(
                autoDriveCommand
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (super.getCurrentCondition() == AutoState.DRIVE_END) {
            return AutoState.DRIVE_END;
        }
        return AutoState.RUNNING;
    }
}
