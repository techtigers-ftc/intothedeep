package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperPreTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import java.util.function.DoubleSupplier;

/**
 * Command to move the intake to prepare to intake state. This command also moves the dropper to the
 * pre-transfer position and opens the intake claw.
 */
public class IntakePrepareToPickupAction extends SequentialCommandGroup {
    private static final String LOG_TAG = IntakePrepareToPickupAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new IntakeToPrepareToIntakeAction
     *
     * @param intake                the intake subsystem
     * @param dropper               the dropper subsystem
     * @param slidePositionSupplier the supplier for the target slide position
     * @param robotState            the robot state
     */
    public IntakePrepareToPickupAction(IntakeSubsystem intake, DropperSubsystem dropper, DoubleSupplier slidePositionSupplier,
                                       RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake, dropper);
        addCommands(
                new ParallelCommandGroup(
                        new IntakeSlidesAbsoluteAction(intake, slidePositionSupplier, 1),
                        new IntakeWristRotationAction(intake,
                                IntakeSubsystem.WRIST_ROTATION_PREPARE_TO_PICKUP_POSITION, 300),
                        new IntakeClawRotationAction(intake,
                                () -> IntakeSubsystem.CLAW_ROTATION_PICKUP_POSITION, 200),
                        new IntakeWristPitchAction(intake,
                                IntakeSubsystem.WRIST_PITCH_PREPARE_TO_PICKUP_POSITION, 200)
                ),
                new ParallelCommandGroup(
                        new DropperPreTransferAction(dropper, robotState),
                        new IntakeOpenAction(intake)
                )
        );
    }

    /**
     * Overloaded constructor that takes a target slide position instead of a supplier
     *
     * @param intake        the intake subsystem
     * @param dropper       the dropper subsystem
     * @param robotState    the robot state
     * @param slidePosition the target slide position
     */
    public IntakePrepareToPickupAction(IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState, double slidePosition) {
        this(intake, dropper, () -> slidePosition, robotState);
    }


    /**
     * Overloaded constructor that keeps the intake slides tucked in. Used for coarse align
     * in order to scan the field
     *
     * @param intake       the intake subsystem
     * @param dropper      the dropper subsystem
     * @param robotState   the robot state
     */
    public IntakePrepareToPickupAction(IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        this(intake, dropper, () -> 0, robotState);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            robotState.setIntakeState(IntakeState.PREPARE_TO_PICKUP);
            if (robotState.getBlockPosition() == RobotBlockPosition.INTAKE) {
                robotState.setBlockPosition(RobotBlockPosition.NONE);
            }
            robotState.setCurrentGear(DriveGears.ENGAGED);
        }
    }
}
