package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperCloseActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperPitchActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperRotationActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperSlidesAbsoluteActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeOpenActionCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * A command group that transfers the sample from the intake to the dropper and
 * moves the dropper to the high chamber drop position, with the specimen
 * upside down, ready to be clipped upwards onto the high chamber
 */
public class DropperToHighChamberTransferAction extends SequentialCommandGroup {
    private final RobotState robotState;

    /**
     * Creates a new DropperToHighChamberTransferAction
     *
     * @param dropper    the dropper subsystem
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public DropperToHighChamberTransferAction(DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper, intake);
        addCommands(
                new DropperCloseActionCommand(dropper, 100),
                new IntakeOpenActionCommand(intake, 200),
                new ParallelCommandGroup(
                        new DropperSlidesAbsoluteActionCommand(dropper, 25, 0.5),
                        new DropperPitchActionCommand(dropper, 300, 300),
                        new DropperRotationActionCommand(dropper, 180, 300)
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        robotState.setDropperState(DropperState.DROP);
    }
}
