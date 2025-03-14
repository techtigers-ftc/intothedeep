package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * An action that moves the dropper to the pre-transfer position. The slides stay up to allow the
 * intake to be tucked inside the robot.
 */
public class DropperPreTransferAction extends ParallelCommandGroup {
    private static final String LOG_TAG = DropperPreTransferAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new DropperPreTransferAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperPreTransferAction(DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperPitchAction(dropper, DropperSubsystem.PITCH_TRANSFER_POSITION, 150),
                new DropperRotationAction(dropper, DropperSubsystem.ROTATION_TRANSFER_POSITION, 150),
                new DropperSlidesAbsoluteAction(dropper, DropperSubsystem.SLIDES_PRE_TRANSFER_POSITION, 0.5),
                new DropperOpenAction(dropper)
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            robotState.setDropperState(DropperState.PRE_TRANSFER);
        }
    }
}
