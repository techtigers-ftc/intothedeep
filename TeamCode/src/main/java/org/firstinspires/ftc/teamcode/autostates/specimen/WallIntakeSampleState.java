package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.drive.RawPowerToDistanceDriveAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * A state to pickup a sample from the wall
 */
public class WallIntakeSampleState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            WallIntakeSampleState.class.getSimpleName();
    private RobotState robotState;
    private boolean dropperPitchUp;

    /**
     * Constructor for the WallIntakeSampleState
     *
     * @param name       The name of the state
     * @param drive      the drive subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public WallIntakeSampleState(String name, DriveSubsystem drive, DropperSubsystem dropper, RobotState robotState) {
        super(name);
        this.robotState = robotState;
        addCommands(
                new RawPowerToDistanceDriveAction(drive, robotState, -0.5, 1.5),
                new DropperCloseAction(dropper, 150),
                new DropperPitchAction(dropper, 295, 10),
                new InstantCommand(() -> dropperPitchUp = true)
        );
    }

    /**
     * Initialize the state, incrementing the run counter
     */
    @Override
    public void initialize() {
        super.initialize();
        dropperPitchUp = false;
    }

    /**
     * Get the current condition of the state
     *
     * @return the current condition of the state based on run counter
     */
    @Override
    public AutoState getCurrentCondition() {
        if (robotState.getDropperClawState() == ClawState.CLOSED && dropperPitchUp) {
            return AutoState.SAMPLE_INTAKE_COMPLETE;
        }
        return AutoState.RUNNING;
    }
}
