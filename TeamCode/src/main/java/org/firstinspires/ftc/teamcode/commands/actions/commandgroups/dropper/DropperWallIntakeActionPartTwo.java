package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

/**
 * A command group that transfers the sample from the intake to the dropper and
 * moves the dropper system to the high basket drop position
 */
public class DropperWallIntakeActionPartTwo extends SequentialCommandGroup {
    private final RobotState robotState;
    private static final String LOG_TAG = DropperWallIntakeActionPartTwo.class.getSimpleName();

    /**
     * Creates a new DropperHighBasketAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperWallIntakeActionPartTwo(DropperSubsystem dropper, RobotState robotState, DriveSubsystem drive) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperPitchAction(dropper, 300, 300),
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
}
