package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

/**
 * A command group that moves the dropper system to the high basket drop position
 * The NT stands for "No Transfer"
 */
public class DropperHighBasketNoTransferAction extends ParallelCommandGroup {
    private final RobotState robotState;
    private static final String LOG_TAG = DropperHighBasketNoTransferAction.class.getSimpleName();

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
                new DropperSlidesAbsoluteAction(dropper, 22, 1),
                new DropperPitchAction(dropper, DropperSubsystem.PITCH_BASKET_POSITION, 300),
                new DropperRotationAction(dropper,
                        DropperSubsystem.ROTATION_BASKET_POSITION, 300)
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
        if (!interrupted){
            robotState.setDropperState(DropperState.HIGH_BASKET);
            robotState.setCurrentGear(DriveGears.ENGAGED);
        }
    }
}
