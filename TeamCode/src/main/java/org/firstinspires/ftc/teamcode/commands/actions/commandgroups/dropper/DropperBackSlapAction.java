package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

/**
 * A command group that moves the dropper slides in order to hang the specimen on the chamber backwards
 */
public class DropperBackSlapAction extends SequentialCommandGroup {
    private final RobotState robotState;
    private static final String LOG_TAG = DropperBackSlapAction.class.getSimpleName();


    /**
     * Creates a new DropperBackSlapAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperBackSlapAction(DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperPitchAction(dropper, DropperSubsystem.PITCH_BACK_SLAP_POSITION, 0),
                new WaitCommand(300)
        );
    }

    @Override
    public void initialize() {
        if (robotState.getDropperState() != DropperState.BACKWARD_CARRY) {
            RobotLog.ww(LOG_TAG, "Invalid dropper position: %s", robotState.getIntakeState());
            robotState.setError(RobotError.INVALID_DROPPER_POSITION);
            this.cancel();
        } else {
            RobotLog.dd(LOG_TAG, "Executing command from state: %s", robotState.getIntakeState());
            robotState.clearError(RobotError.INVALID_DROPPER_POSITION);
            super.initialize();
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            robotState.setDropperState(DropperState.BACK_SLAP);
        }
    }
}
