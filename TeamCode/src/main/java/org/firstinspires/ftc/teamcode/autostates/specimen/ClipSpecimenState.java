package org.firstinspires.ftc.teamcode.autostates.specimen;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperBackSlapAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * A state to clip a specimen onto the chamber
 */
public class ClipSpecimenState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            ClipSpecimenState.class.getSimpleName();
    private int runCounter;
    private RobotState robotState;

    /**
     * Constructor for the ClipSpecimenState
     *
     * @param name    The name of the state
     * @param dropper The dropper subsystem
     * @param robotState The robot state
     */
    public ClipSpecimenState(String name, DropperSubsystem dropper, RobotState robotState) {
        super(name);
        this.robotState = robotState;
        runCounter = 0;
        addCommands(
                new DropperRotationAction(dropper, DropperSubsystem.ROTATION_BACK_SLAP_POSITION, 200),
                new DropperBackSlapAction(dropper, robotState),
                new DropperOpenAction(dropper)
        );
    }

    /**
     * Initialize the state, incrementing the run counter
     */
    @Override
    public void initialize() {
        runCounter++;
        super.initialize();
    }

    /**
     * Get the current condition of the state
     *
     * @return the current condition of the state based on run counter
     */
    @Override
    public AutoState getCurrentCondition() {
        if (robotState.getDropperClawState() == ClawState.OPEN) {
            if (runCounter == 1) {
                return AutoState.SPECIMEN_1_DROP_COMPLETE;
            } else if (runCounter == 2) {
                return AutoState.SPECIMEN_2_DROP_COMPLETE;
            } else if (runCounter == 3) {
                return AutoState.SPECIMEN_3_DROP_COMPLETE;
            }
            return AutoState.SPECIMEN_4_DROP_COMPLETE;
        }
        return AutoState.RUNNING;
    }
}
