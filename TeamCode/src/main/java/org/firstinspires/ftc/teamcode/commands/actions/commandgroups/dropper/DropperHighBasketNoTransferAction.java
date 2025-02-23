package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * A command group that moves the dropper system to the high basket drop position
 * The NT stands for "No Transfer"
 */
public class DropperHighBasketNoTransferAction extends ParallelCommandGroup {
    private static final String LOG_TAG = DropperHighBasketNoTransferAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new DropperHighBasketNTAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperHighBasketNoTransferAction(DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperSlidesAbsoluteAction(dropper,
                        DropperSubsystem.SLIDE_MAX, 1.5),
                new DropperRotationAction(dropper,
                        DropperSubsystem.ROTATION_BASKET_POSITION, 300),
                new SequentialCommandGroup(
                        new DropperPitchAction(dropper, 180, 300),
                        new WaitUntilCommand(() -> dropper.getCurrentSlidePositionInches() > 20),
                        new DropperPitchAction(dropper, DropperSubsystem.PITCH_BASKET_POSITION, 300)
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            robotState.setDropperState(DropperState.HIGH_BASKET);
        }
    }
}
