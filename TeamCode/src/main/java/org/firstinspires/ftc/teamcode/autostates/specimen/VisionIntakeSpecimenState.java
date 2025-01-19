package org.firstinspires.ftc.teamcode.autostates.specimen;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.IntakeVisionPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToTransferAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * A state to pick up a specimen from the observation zone using the vision
 */
public class VisionIntakeSpecimenState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            VisionIntakeSpecimenState.class.getSimpleName();
    private final RobotState robotState;
    private int runCounter;

    /**
     * Constructor for the VisionIntakeSpecimenState
     *
     * @param name       The name of the state
     * @param intake     The intake subsystem
     * @param dropper    The dropper subsystem
     * @param drive      The drive subsystem
     * @param robotState The robot state
     */
    public VisionIntakeSpecimenState(String name, IntakeSubsystem intake, DropperSubsystem dropper, DriveSubsystem drive,
                                     RobotState robotState) {
        super(name);
        this.robotState = robotState;
        runCounter = 0;
        addCommands(
                new IntakeVisionPickupAction(intake, dropper, drive, robotState, () -> robotState.getRobotCurrentPose().getHeading()),
                new IntakePrepareToTransferAction(intake, dropper, robotState),
                new IntakeReadyToTransferAction(intake, robotState)
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
     * Get the current condition of the robot
     *
     * @return the current condition of the robot using the AutoState enum
     */
    @Override
    public AutoState getCurrentCondition() {
        if (robotState.getIntakeState() == IntakeState.READY_TO_TRANSFER) {
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
}
