package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A command group that transfers the sample from the intake to the dropper and
 * moves the dropper system to the high basket drop position
 */
public class DropperHighBasketAction extends SequentialCommandGroup {
    private final RobotState robotState;
    private final IntakeSubsystem intake;

    /**
     * Creates a new DropperHighBasketAction
     *
     * @param dropper    the dropper subsystem
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public DropperHighBasketAction(DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        this.intake = intake;
        addRequirements(dropper, intake);
        addCommands(
                new DropperTransferAction(dropper, intake, robotState),
                new DropperHighBasketNTAction(dropper, robotState)
        );
    }
}
