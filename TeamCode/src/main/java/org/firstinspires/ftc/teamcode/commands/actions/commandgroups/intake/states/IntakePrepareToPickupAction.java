package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

/**
 * Command to move the intake to prepare to intake state. This command also moves the dropper to the
 * pre-transfer position and opens the intake claw.
 */
public class IntakePrepareToPickupAction extends ParallelCommandGroup {
    private static final String LOG_TAG = IntakePrepareToPickupAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new IntakeToPrepareToIntakeAction
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public IntakePrepareToPickupAction(IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new IntakeWristRotationAction(intake, IntakeSubsystem.WRIST_ROTATION_PREPARE_TO_PICKUP_POSITION, 300),
                new IntakeClawRotationAction(intake, () -> IntakeSubsystem.CLAW_ROTATION_PICKUP_POSITION, 200),
                new IntakeWristPitchAction(intake, IntakeSubsystem.WRIST_PITCH_PREPARE_TO_PICKUP_POSITION, 200),
                new IntakeSlidesAbsoluteAction(intake, () -> 0, 1),
                new IntakeOpenAction(intake)
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        robotState.setCoarseCameraMode(true);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            robotState.setCoarseCameraMode(true);
            robotState.setIntakeState(IntakeState.PREPARE_TO_PICKUP);
            if (robotState.getBlockPosition() == RobotBlockPosition.INTAKE) {
                robotState.setBlockPosition(RobotBlockPosition.NONE);
            }
            robotState.setCurrentGear(DriveGears.ENGAGED);
        }
    }
}
