package org.firstinspires.ftc.teamcode.autostates.visiontest;


import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * A state to transfer the sample from the intake and bring it back to the wall intake position
 */
public class DropperWallIntakeAndDropState extends SequentialCommandGroupState<AutoState> {
    private final RobotState robotState;

    /**
     * Constructor for the DropperWallIntakeAndDropState
     *
     * @param name       The name of the state
     * @param intake     The intake subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public DropperWallIntakeAndDropState(String name, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        super(name, 1.5);
        this.robotState = robotState;

        addCommands(
                new WaitCommand(200),
                new DropperWallIntakeAction(dropper, intake, robotState)
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (robotState.getDropperState() == DropperState.WALL_INTAKE) {
            return AutoState.DRIVE_END;
        }
        return AutoState.RUNNING;
    }
}
