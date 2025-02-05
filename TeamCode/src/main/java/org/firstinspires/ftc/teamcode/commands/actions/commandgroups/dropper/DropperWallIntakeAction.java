package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.TransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.commands.actions.drive.TeleHoldPointAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

/**
 * A command group that transfers the sample from the intake to the dropper and
 * moves the dropper system to the high basket drop position
 */
public class DropperWallIntakeAction extends ParallelCommandGroup {
    private final RobotState robotState;
    private static final String LOG_TAG = DropperWallIntakeAction.class.getSimpleName();

    /**
     * Creates a new DropperHighBasketAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperWallIntakeAction(DropperSubsystem dropper, RobotState robotState, DriveSubsystem drive) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperSlidesAbsoluteAction(dropper, DropperSubsystem.SLIDES_WALL_INTAKE_POSITION, 0.5),
                new DropperPitchAction(dropper, DropperSubsystem.PITCH_WALL_INTAKE_POSITION, 200),
                new DropperRotationAction(dropper,
                        DropperSubsystem.ROTATION_BASKET_POSITION, 200),
                new DropperOpenAction(dropper, 100)
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
            robotState.setCurrentGear(DriveGears.ENGAGED);
            robotState.setDropperState(DropperState.WALL_INTAKE);
        }
    }
}
