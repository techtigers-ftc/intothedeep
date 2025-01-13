package org.firstinspires.ftc.teamcode.autostates.specimen;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeOpenAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * A state to open the intake claw, used in the specimen auto to drop samples into the observation zone
 */
public class OpenIntakeState extends SequentialCommandGroupState<AutoState> {
    private final RobotState robotState;
    private static final String LOG_TAG =
            OpenIntakeState.class.getSimpleName();

    /**
     * Constructor for the OpenIntakeState
     *
     * @param name The name of the state
     * @param robotState The robot state
     * @param intakeSubsystem The intake subsystem
     */
    public OpenIntakeState(String name, RobotState robotState, IntakeSubsystem intakeSubsystem) {
        super(name);
        this.robotState = robotState;
        addCommands(
                new IntakeOpenAction(intakeSubsystem)
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (robotState.getIntakeClawState() == ClawState.OPEN) {
            return AutoState.SAMPLE_1_DROP_COMPLETE;
        }
        return AutoState.RUNNING;
    }
}
