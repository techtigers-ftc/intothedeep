package org.firstinspires.ftc.teamcode.autostates.specimen;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperHighBasketAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * Drive state that drives the robot to a sample drop from an intake
 */
public class GeneralDriveToPush extends DriveStateBase {

    /**
     * Constructor for the SequentialCommandGroupState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param robotState The robot state
     */
    public GeneralDriveToPush(String name, DriveSubsystem drive, RobotState robotState) {
        super(name, drive, robotState);

        addCommands(
                autoDriveCommand
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (super.getCurrentCondition() == AutoState.DRIVE_END ) {
            return AutoState.DRIVE_END;
        }
        return AutoState.RUNNING;
    }
}
