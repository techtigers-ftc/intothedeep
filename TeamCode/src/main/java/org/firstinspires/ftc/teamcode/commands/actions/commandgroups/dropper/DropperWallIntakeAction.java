package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.TransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

/**
 * A command group that transfers the sample from the intake to the dropper and
 * moves the dropper system to the high basket drop position
 */
public class DropperWallIntakeAction extends SequentialCommandGroup {
    private final RobotState robotState;
    private static final String LOG_TAG = DropperWallIntakeAction.class.getSimpleName();

    /**
     * Creates a new DropperHighBasketAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperWallIntakeAction(DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new ParallelCommandGroup(
                        new DropperPitchAction(dropper, 315, 200),
                        new DropperRotationAction(dropper, DropperSubsystem.ROTATION_BASKET_POSITION, 200)
                ),
                new DropperOpenAction(dropper, 200),
                new DropperPitchAction(dropper, 300, 200)

        );
    }
    @Override
    public void initialize() {
        RobotLog.dd(LOG_TAG, "Executing command from state: %s", robotState.getIntakeState());
        robotState.clearError(RobotError.INVALID_DROPPER_POSITION);
        super.initialize();
    }
}
