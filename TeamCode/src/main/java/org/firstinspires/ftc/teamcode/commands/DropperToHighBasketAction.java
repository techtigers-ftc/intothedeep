package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeOpenAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

/**
 * A command group that transfers the sample from the intake to the dropper and
 * moves the dropper system to the high basket drop position
 */
public class DropperToHighBasketAction extends SequentialCommandGroup {
    private final RobotState robotState;

    /**
     * Creates a new DropperToHighBasketDropCommandGroup
     *
     * @param dropper the dropper subsystem
     * @param intake  the intake subsystem
     */
    public DropperToHighBasketAction(DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperCloseAction(dropper, 100),
                new IntakeOpenAction(intake, 200),
                new ParallelCommandGroup(
                        new DropperSlidesAbsoluteAction(dropper, 25, 0.5),
                        new DropperPitchAction(dropper, 0, 300),
                        new DropperRotationAction(dropper, 180, 300)
                        // Drop forward: 1
                        // Pick up from intake: 0.6
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        robotState.setDropperState(DropperState.DROP);
        robotState.setBlockPosition(RobotBlockPosition.DROPPER);
    }
}
