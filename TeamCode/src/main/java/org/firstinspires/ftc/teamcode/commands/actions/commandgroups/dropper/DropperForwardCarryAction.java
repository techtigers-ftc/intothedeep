package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

/**
 * A command group that transfers the sample from the intake to the dropper and
 * moves the dropper to the forward high chamber drop position, with the specimen
 * upside down, ready to be clipped downwards onto the high chamber
 */
public class DropperForwardCarryAction extends SequentialCommandGroup {
    private static final String LOG_TAG = DropperForwardCarryAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new DropperForwardCarryAction
     *
     * @param dropper    the dropper subsystem
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public DropperForwardCarryAction(DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper, intake);
        addCommands(
                new DropperTransferAction(dropper, intake, robotState),
                new ParallelCommandGroup(
                        new DropperForwardCarryNoTransferAction(dropper, robotState),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new IntakeTuckAction(intake, robotState)
                        )
                )
        );
    }

    @Override
    public void initialize() {
        RobotLog.dd(LOG_TAG, "Executing command from state: %s", robotState.getIntakeState());
        robotState.clearError(RobotError.INVALID_DROPPER_POSITION);
        super.initialize();
    }
}
