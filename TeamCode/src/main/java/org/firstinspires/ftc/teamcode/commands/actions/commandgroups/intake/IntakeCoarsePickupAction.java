package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * An action to use the limelight to roughly align the robot to a region of samples.
 * This action will move the robot to the region of samples and position the intake over the blocks.
 * It will then run the fine pickup to pickup the block
 */
public class IntakeCoarsePickupAction extends SequentialCommandGroup {
    private static final String LOG_TAG = IntakeCoarsePickupAction.class.getSimpleName();

    /**
     * Creates a new IntakeCoarsePickupAction
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public IntakeCoarsePickupAction(DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        addRequirements(drive, intake, dropper);
        addCommands(
                new WaitUntilCommand(robotState::isBlockDetected),
                new IntakeCoarseAlignAction(drive, intake, robotState),
                new WaitUntilCommand(robotState::isBlockDetected),
                new IntakeFullReadyToTransferAction(drive, intake, dropper, robotState)
        );
    }
}
