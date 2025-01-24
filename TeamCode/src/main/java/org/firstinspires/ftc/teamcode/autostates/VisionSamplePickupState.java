package org.firstinspires.ftc.teamcode.autostates;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.IntakeVisionPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToTransferAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * State for using the vision system to pick a sample out from the submersible
 */
public class VisionSamplePickupState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG = VisionSamplePickupState.class.getSimpleName();
    private final RobotState robotState;
    private int runCounter;

    /**
     * Creates a new VisionSamplePickupState
     *
     * @param name the name of the state
     * @param intake the intake subsystem
     * @param dropper the dropper subsystem
     * @param drive the drive subsystem
     * @param robotState the robot state
     */
    public VisionSamplePickupState(String name, IntakeSubsystem intake, DropperSubsystem dropper, DriveSubsystem drive, RobotState robotState) {
        super(name);
        this.robotState = robotState;
        runCounter = 0;
        addCommands(
                new IntakeVisionPickupAction(intake, dropper, drive, robotState, robotState::getVisionIntakeHeading),
                new IntakePrepareToTransferAction(intake, dropper, () -> 5, robotState),
                new IntakeReadyToTransferAction(intake, robotState)
        );
    }

    @Override
    public void initialize() {
        runCounter++;
        super.initialize();
    }

    @Override
    public AutoState getCurrentCondition() {
        if (robotState.getIntakeState() == IntakeState.READY_TO_TRANSFER) {
            if(runCounter == 1) {
                return AutoState.SAMPLE_5_INTAKE_COMPLETE;
            } else {
                return AutoState.SAMPLE_6_INTAKE_COMPLETE;
            }
        } else {
            return AutoState.RUNNING;
        }
    }
}
