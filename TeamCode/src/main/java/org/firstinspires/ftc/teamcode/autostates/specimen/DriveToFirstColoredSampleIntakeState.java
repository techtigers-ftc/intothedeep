package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

public class DriveToFirstColoredSampleIntakeState extends DriveStateBase {
    public DriveToFirstColoredSampleIntakeState(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState, double timeout) {
        super(name, drive, robotState, timeout);
        addCommands(
            autoDriveCommand,
            new DropperWallIntakeNoTransferAction(dropper, robotState),
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> robotState.getRobotCurrentPose().getX() > 0), //TODO: Find the right X
                new IntakeReadyToPickupAction(intake, robotState, () -> 10) //TODO: Find the right slide position
            )
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if(super.getCurrentCondition() == AutoState.TIMEOUT) {
            return AutoState.TIMEOUT;
        } else if(super.getCurrentCondition() == AutoState.DRIVE_END && robotState.getIntakeState() == IntakeState.READY_TO_PICKUP) {
            return AutoState.DRIVE_END;
        }
        return AutoState.RUNNING;
    }
}
