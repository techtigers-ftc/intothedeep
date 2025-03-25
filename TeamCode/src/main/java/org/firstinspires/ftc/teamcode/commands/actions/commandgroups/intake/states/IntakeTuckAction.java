package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

/**
 * Command to put the intake into the tuck position
 */
public class IntakeTuckAction extends ParallelCommandGroup {
    private static final String LOG_TAG = IntakeTuckAction.class.getSimpleName();
    private final IntakeSubsystem intake;
    private final RobotState robotState;

    /**
     * Creates a new IntakeTuckCommand
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public IntakeTuckAction(IntakeSubsystem intake, RobotState robotState) {
        this.intake = intake;
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new IntakeSlidesAbsoluteAction(intake, () -> 0, 0.75),
                new IntakeWristPitchAction(intake, IntakeSubsystem.WRIST_PITCH_TUCK_POSITION, 200),
                new IntakeClawRotationAction(intake, () -> IntakeSubsystem.CLAW_ROTATION_TUCK_POSITION, 200),
                new IntakeWristRotationAction(intake, IntakeSubsystem.WRIST_ROTATION_TUCK_POSITION, 200),
                new IntakeCloseAction(intake)
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (interrupted) {
            intake.setWristAbsolute(IntakeSubsystem.WRIST_PITCH_TUCK_POSITION,
                    IntakeSubsystem.WRIST_ROTATION_TUCK_POSITION);
            intake.setClawRotationAbsolute(IntakeSubsystem.CLAW_ROTATION_TUCK_POSITION);
        }
        robotState.setIntakeState(IntakeState.TUCK);
        robotState.setCurrentGear(DriveGears.NOT_ENGAGED);
    }
}
