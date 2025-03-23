package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFineAlignAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

import team.techtigers.base.statemachine.ParallelCommandGroupState;

public class IntakeAndDropSecondColoredSampleState extends ParallelCommandGroupState<AutoState> {
    RobotState robotState;
    public IntakeAndDropSecondColoredSampleState(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        super(name);
        addCommands(
            new SequentialCommandGroup(
                    new WaitUntilCommand(robotState::hasBlockBeenDetected),
                    new IntakeFineAlignAction(drive, intake, () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading()), robotState)
            ),
            new SequentialCommandGroup(
                new DropperWallIntakeNoTransferAction(dropper, robotState),
                new DropperOpenAction(dropper, 100)
            )
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (robotState.getIntakeState() == IntakeState.READY_TO_TRANSFER && robotState.getDropperClawState() == ClawState.OPEN) {
            return AutoState.SAMPLE_INTAKE_COMPLETE;
        }
        return AutoState.RUNNING;
    }
}
