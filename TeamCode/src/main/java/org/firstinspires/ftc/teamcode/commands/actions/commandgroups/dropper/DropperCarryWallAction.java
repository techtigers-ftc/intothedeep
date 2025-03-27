package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperCloseAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

/**
 * A command group that closes the dropper and moves the dropper to forward carry
 */
public class DropperCarryWallAction extends SequentialCommandGroup {
    private final RobotState robotState;
    private static final String LOG_TAG = DropperCarryWallAction.class.getSimpleName();

    /**
     * Creates a new DropperCarryWallAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperCarryWallAction(DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperCloseAction(dropper, 150),
                new DropperCarryNoTransferAction(dropper, robotState)
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            robotState.setBlockPosition(RobotBlockPosition.DROPPER);
            robotState.setCurrentGear(DriveGears.NOT_ENGAGED);
        }
    }
}
