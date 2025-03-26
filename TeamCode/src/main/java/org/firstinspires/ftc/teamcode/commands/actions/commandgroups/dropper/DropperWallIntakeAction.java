package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTuckAfterTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.TransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

/**
 * A command group that transfers a sample and gets the robot ready to do a wall intake
 */
public class DropperWallIntakeAction extends SequentialCommandGroup {
    private static final String LOG_TAG = DropperWallIntakeAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new DropperWallIntakeAction
     *
     * @param dropper    the dropper subsystem
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public DropperWallIntakeAction(DropperSubsystem dropper,
                                   IntakeSubsystem intake,
                                   RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new TransferAction(dropper, intake, robotState),
//                new ParallelCommandGroup(
//                        new IntakeTuckAfterTransferAction(dropper, intake,
//                                robotState),
//                        new DropperSlidesAbsoluteAction(dropper, 6,
//                                0.5),
//                        new DropperPitchAction(dropper,
//                                DropperSubsystem.PITCH_WALL_INTAKE_POSITION, 200),
//                        new DropperRotationAction(dropper,
//                                DropperSubsystem.ROTATION_WALL_INTAKE_POSITION, 200)
//                ),
//                new DropperSlidesAbsoluteAction(dropper,
//                        DropperSubsystem.SLIDES_WALL_INTAKE_POSITION, 0.5)
                new IntakeWristPitchAction(intake,
                        IntakeSubsystem.WRIST_PITCH_TRANSFER_POSITION + 20, 0),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new IntakeTuckAction(intake, robotState)
                        ),
                        new DropperWallIntakeNoTransferAction(dropper, robotState)
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            robotState.setDropperState(DropperState.WALL_INTAKE);
        }
    }
}
