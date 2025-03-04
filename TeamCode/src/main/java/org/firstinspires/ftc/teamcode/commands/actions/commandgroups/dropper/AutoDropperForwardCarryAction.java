package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

/**
 * A command group that moves the dropper to forward carry for the autonomous. It forgoes the
 */
public class AutoDropperForwardCarryAction extends ParallelCommandGroup {
    private static final String LOG_TAG = AutoDropperForwardCarryAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new AutoDropperForwardCarryAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public AutoDropperForwardCarryAction(DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperSlidesAbsoluteAction(dropper, DropperSubsystem.SLIDES_CHAMBER_POSITION, 0.5),
                new DropperPitchAction(dropper, DropperSubsystem.AUTO_PITCH_CHAMBER_POSITION, 300),
                new DropperRotationAction(dropper,
                        DropperSubsystem.ROTATION_FRONT_SLAP_POSITION, 300)
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            robotState.setBlockPosition(RobotBlockPosition.DROPPER);
            robotState.setDropperState(DropperState.FORWARD_CARRY);
            robotState.setCurrentGear(DriveGears.NOT_ENGAGED);
        }
    }
}
