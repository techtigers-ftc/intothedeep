package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

/**
 * A command group that transfers the sample from the intake to the dropper and
 * moves the dropper system to the high basket drop position
 */
public class DropperForwardCarryWallAction extends SequentialCommandGroup {
    private final RobotState robotState;
    private static final String LOG_TAG = DropperForwardCarryWallAction.class.getSimpleName();

    /**
     * Creates a new DropperHighBasketAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperForwardCarryWallAction(DropperSubsystem dropper, RobotState robotState, DriveSubsystem drive) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperCloseAction(dropper, 200),
                new DropperForwardCarryNoTransferAction(dropper, robotState)
        );
    }

    @Override
    public void initialize() {
        RobotLog.dd(LOG_TAG, "Executing command from state: %s", robotState.getIntakeState());
        robotState.clearError(RobotError.INVALID_DROPPER_POSITION);
        super.initialize();
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
