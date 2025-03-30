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
        super(name);
        this.robotState = robotState;
        this.dropper = dropper;
        runCounter = 0;
        addCommands(
//                new WaitCommand(500000),
                new RawPowerToDistanceDriveAction(drive, robotState, -0.4, 1.5),
                new DropperCloseAction(dropper, 0),
                new DropperPitchAction(dropper, 275, 0),
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
        if (dropperPitchUp) {
            if (runCounter == 1) {
                return AutoState.SPECIMEN_1_INTAKE_COMPLETE;
            } else if (runCounter == 2) {
                return AutoState.SPECIMEN_2_INTAKE_COMPLETE;
            } else if (runCounter == 3) {
                return AutoState.SPECIMEN_3_INTAKE_COMPLETE;
            } else if (runCounter == 4) {
                return AutoState.SPECIMEN_4_INTAKE_COMPLETE;
            } else {
                return AutoState.SPECIMEN_5_INTAKE_COMPLETE;
            }
        }
        return AutoState.RUNNING;
    }
}
