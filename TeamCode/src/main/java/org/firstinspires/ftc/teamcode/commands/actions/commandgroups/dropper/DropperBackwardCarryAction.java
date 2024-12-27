package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

/**
 * A command group that transfers the sample from the intake to the dropper and
 * moves the dropper to the backward high chamber drop position, with the specimen
 * upside down, ready to be clipped upwards onto the high chamber
 */
public class DropperBackwardCarryAction extends ParallelCommandGroup {
    private final RobotState robotState;

    /**
     * Creates a new DropperBackwardHighChamberAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperBackwardCarryAction(DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperPitchAction(dropper, DropperSubsystem.PITCH_CHAMBER_POSITION, 300),
                new DropperRotationAction(dropper,
                        DropperSubsystem.ROTATION_BACK_SLAP_POSITION, 300)
        );
    }

    @Override
    public void end(boolean interrupted) {
        robotState.setDropperState(DropperState.BACKWARD_CARRY);
    }
}
