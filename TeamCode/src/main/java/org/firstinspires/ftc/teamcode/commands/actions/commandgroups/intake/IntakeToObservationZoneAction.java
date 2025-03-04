package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperPreTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

/**
 * Command to move intake to observation zone. This command sends the slides out and puts the intake
 * into the prepare to pickup position.
 */
public class IntakeToObservationZoneAction extends ParallelCommandGroup {
    private static final String LOG_TAG = IntakeToObservationZoneAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new IntakeToPrepareToIntakeCommand
     *
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public IntakeToObservationZoneAction(IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new DropperPreTransferAction(dropper, robotState),
                new IntakeSlidesAbsoluteAction(intake, () -> 5, 1),
                new IntakeWristRotationAction(intake,
                        IntakeSubsystem.WRIST_ROTATION_PREPARE_TO_PICKUP_POSITION, 300),
                new SequentialCommandGroup(
                        new IntakeClawRotationAction(intake, () -> 30, 0),
                        new WaitUntilCommand(() -> intake.getWristRotation() > 30),
                        new IntakeClawRotationAction(intake, () -> IntakeSubsystem.CLAW_ROTATION_PICKUP_POSITION, 100)
                ),
                new IntakeWristPitchAction(intake,
                        IntakeSubsystem.WRIST_PITCH_PREPARE_TO_PICKUP_POSITION, 300)
        );
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
            robotState.setCurrentGear(DriveGears.NOT_ENGAGED);
        }
    }
}
