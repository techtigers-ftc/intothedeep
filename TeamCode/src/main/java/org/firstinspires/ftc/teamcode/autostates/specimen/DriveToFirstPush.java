package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperPreTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * A State to drive to the submersible position
 */
public class DriveToFirstPush extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToFirstPush.class.getSimpleName();
    private static final double TOLERANCE = 1.5;
    private static final double ANGULAR_TOLERANCE = Math.toRadians(5);

    /**
     * Constructor for the DriveToSubmersible
     *
     * @param name The name of the state
     * @param drive The drive subsystem
     * @param dropper The dropper subsystem
     * @param intake The intake subsystem
     * @param robotState The robot state
     */
    public DriveToFirstPush(String name, DriveSubsystem drive, DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
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
