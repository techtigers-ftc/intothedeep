package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import java.util.function.DoubleSupplier;

/**
 * Command to align to a block using fine camera vision
 */
public class IntakeFinePickupAction extends SequentialCommandGroup {
    private static final String LOG_TAG = IntakePrepareToPickupAction.class.getSimpleName();
    private final RobotState robotState;
    private final IntakeSubsystem intake;

    /**
     * Creates a new IntakeFineAlignAction and allows you to specify the claw rotation
     *
     * @param drive                the drive subsystem
     * @param intake               the intake subsystem
     * @param clawRotationSupplier the supplier for the claw rotation
     * @param robotState           the robot state
     */
    public IntakeFinePickupAction(DriveSubsystem drive, IntakeSubsystem intake,
                                  DoubleSupplier clawRotationSupplier,
                                  RobotState robotState) {
        this.robotState = robotState;
        this.intake = intake;
        addRequirements(intake);
        addCommands(
                new IntakeFineAlignAction(drive, intake, clawRotationSupplier, robotState),
                new IntakePrepareToTransferAction(intake, robotState)
        );
    }

    /**
     * Creates a new IntakeFineAlignAction and has the robot calculate block orientation
     *
     * @param drive                the drive subsystem
     * @param intake               the intake subsystem
     * @param robotState           the robot state
     */
    public IntakeFinePickUpAction(DriveSubsystem drive, IntakeSubsystem intake,
                                  RobotState robotState) {
        this(drive, intake, null, robotState);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        robotState.setVisionAligning(false);
    }
}
