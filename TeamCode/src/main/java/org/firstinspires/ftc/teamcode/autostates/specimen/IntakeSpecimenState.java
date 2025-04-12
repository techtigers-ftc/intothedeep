package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.drive.RawPowerToDistanceDriveAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * A state to pickup a specimen from the wall
 */
public class IntakeSpecimenState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            IntakeSpecimenState.class.getSimpleName();
    private int runCounter;
    private RobotState robotState;
    private DropperSubsystem dropper;
    private boolean dropperPitchUp;

    /**
     * Constructor for the IntakeSpecimenState
     *
     * @param name       The name of the state
     * @param drive      the drive subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public IntakeSpecimenState(String name, DriveSubsystem drive, DropperSubsystem dropper, RobotState robotState) {
        super(name, 1);
        this.robotState = robotState;
        this.dropper = dropper;
        runCounter = 0;
        addCommands(
//                new WaitCommand(500000),
                new RawPowerToDistanceDriveAction(drive, robotState, -0.3, 2.5),
                new DropperCloseAction(dropper, 0),
                new DropperPitchAction(dropper, 240, 20),
//                new WaitCommand(10),
                new InstantCommand(() -> dropperPitchUp = true)
        );
    }

    /**
     * Initialize the state, incrementing the run counter
     */
    @Override
    public void initialize() {
        runCounter++;
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
        if (isTimeoutReached()) {
            return AutoState.TIMEOUT;
        } if (dropperPitchUp) {
            return AutoState.SPECIMEN_INTAKE_COMPLETE;
        }
        return AutoState.RUNNING;
    }
}
