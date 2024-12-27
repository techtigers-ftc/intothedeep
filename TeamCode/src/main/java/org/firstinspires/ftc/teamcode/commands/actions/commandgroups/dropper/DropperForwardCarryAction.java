package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A command group that transfers the sample from the intake to the dropper and
 * moves the dropper to the forward high chamber drop position, with the specimen
 * upside down, ready to be clipped downwards onto the high chamber
 */
public class DropperForwardCarryAction extends SequentialCommandGroup {

    /**
     * Creates a new DropperForwardCarryAction
     *
     * @param dropper    the dropper subsystem
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public DropperForwardCarryAction(DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        addRequirements(dropper, intake);
        addCommands(
                new DropperTransferAction(dropper, intake, robotState),
                new DropperForwardCarryNoTransferAction(dropper, robotState),
                new IntakeTuckAction(intake, robotState)
        );
    }
}
