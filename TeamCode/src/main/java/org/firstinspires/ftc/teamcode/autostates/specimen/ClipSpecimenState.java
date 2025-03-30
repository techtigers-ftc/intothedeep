package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.drive.RawPowerDriveAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;

import team.techtigers.base.statemachine.ParallelCommandGroupState;

/**
 * A state to clip a specimen onto the chamber
 */
public class ClipSpecimenState extends ParallelCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            ClipSpecimenState.class.getSimpleName();
    private int runCounter;
    private RobotState robotState;

    /**
     * Constructor for the ClipSpecimenState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public ClipSpecimenState(String name, DriveSubsystem drive, DropperSubsystem dropper, RobotState robotState) {
        super(name, 0.4);
        this.robotState = robotState;
        runCounter = 0;
        addCommands(
                new RawPowerDriveAction(drive, 0.8, 0.75),
                new SequentialCommandGroup(
                        new WaitCommand(25),
                        new DropperPitchAction(dropper,
                                DropperSubsystem.PITCH_SLAP_POSITION, 0),
                        new WaitCommand(50),
                        new DropperOpenAction(dropper)
                )
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
        if (robotState.getDropperClawState() == ClawState.OPEN || isTimeoutReached()) {
            if (runCounter == 1) {
                return AutoState.SPECIMEN_PRELOAD_DROP_COMPLETE;
            } else if (runCounter == 2) {
                return AutoState.SPECIMEN_1_DROP_COMPLETE;
            } else if (runCounter == 3) {
                return AutoState.SPECIMEN_2_DROP_COMPLETE;
            } else if (runCounter == 4) {
                return AutoState.SPECIMEN_3_DROP_COMPLETE;
            } else {
                return AutoState.SPECIMEN_4_DROP_COMPLETE;
            }
        }
        return AutoState.RUNNING;
    }
}
