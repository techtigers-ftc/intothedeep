package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.IntakeTuckAfterTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.TransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

/**
 * A command group that Prepare the Dropper for wall intake
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
                new ParallelCommandGroup(
                        new IntakeTuckAfterTransferAction(dropper, intake,
                                robotState),
                        new DropperSlidesAbsoluteAction(dropper, 6,
                                0.5),
                        new DropperPitchAction(dropper,
                                DropperSubsystem.PITCH_WALL_INTAKE_POSITION, 200),
                        new DropperRotationAction(dropper,
                                DropperSubsystem.ROTATION_WALL_INTAKE_POSITION, 200)
                ),
                new DropperSlidesAbsoluteAction(dropper,
                        DropperSubsystem.SLIDES_WALL_INTAKE_POSITION, 0.5)
        );
    }

    @Override
    public void initialize() {
        robotState.clearError(RobotError.INVALID_DROPPER_POSITION);
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            robotState.setDropperState(DropperState.WALL_INTAKE);
        }
    }
}
