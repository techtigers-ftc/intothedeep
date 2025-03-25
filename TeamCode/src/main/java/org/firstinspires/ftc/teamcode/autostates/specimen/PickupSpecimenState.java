package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.drive.RawPowerToDistanceDriveAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * A state to pickup a specimen from the wall
 */
public class PickupSpecimenState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            PickupSpecimenState.class.getSimpleName();
    private int runCounter;
    private RobotState robotState;
    private DropperSubsystem dropper;
    private final ElapsedTime timer;

    /**
     * Constructor for the PickupSpecimenState
     *
     * @param name       The name of the state
     * @param drive      the drive subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public PickupSpecimenState(String name, DriveSubsystem drive, DropperSubsystem dropper, RobotState robotState) {
        super(name);
        this.robotState = robotState;
        this.dropper = dropper;
        runCounter = 0;
        timer = new ElapsedTime();
        addCommands(
                new RawPowerToDistanceDriveAction(drive, robotState, -0.4, 1.5),
                new DropperCloseAction(dropper, 150),
                new DropperSlidesAbsoluteAction(dropper, 2.5, 1)
        );
    }

    /**
     * Initialize the state, incrementing the run counter
     */
    @Override
    public void initialize() {
        runCounter++;
        super.initialize();
        timer.reset();
    }

    /**
     * Get the current condition of the state
     *
     * @return the current condition of the state based on run counter
     */
    @Override
    public AutoState getCurrentCondition() {
        if (robotState.getDropperClawState() == ClawState.CLOSED && dropper.getCurrentSlidePositionInches() > 1.5) {
            if (runCounter == 1) {
                return AutoState.SPECIMEN_1_INTAKE_COMPLETE;
            } else if (runCounter == 2) {
                return AutoState.SPECIMEN_2_INTAKE_COMPLETE;
            } else if (runCounter == 3) {
                return AutoState.SPECIMEN_3_INTAKE_COMPLETE;
            }
            return AutoState.SPECIMEN_4_INTAKE_COMPLETE;
        }
        return AutoState.RUNNING;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        RobotLog.dd("Specimen Auto Debug", "Pickup State Time to End: %f", timer.seconds());
    }
}
