package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.SetFineCameraOrientationAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Command to move intake to ready to pickup state, tracking the block and its orientation with the small camera
 * If the block is in the intake, it will move to the ready to transfer state
 */
public class SmallCameraVisionPickup extends SequentialCommandGroup {
    private static final String LOG_TAG = SmallCameraVisionPickup.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new AutoIntakeReadyToPickupCommand
     *
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     * @param command    the command to cancel using the intake color sensor
     */
    public SmallCameraVisionPickup(IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState, CommandBase command) {
        this.robotState = robotState;
        addCommands(
                new SetFineCameraOrientationAction(robotState),
                new IntakeReadyToPickupAction(intake, robotState,
                        () -> intake.getCurrentSlidePositionInches() - robotState.getBlockForwardFine() - VisionSubsystem.INTAKE_CAMERA_OFFSET,
                        robotState::getBlockOrientation)
                // TODO: Fix this
//                ,
//                new IntakeFullReadyToTransferAction(intake, dropper, robotState, command == null ? this : command)
        );
    }
}
