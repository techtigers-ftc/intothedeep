package org.firstinspires.ftc.teamcode.autostates.basket;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTrackingAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * State for using the vision system to pick a sample out from the submersible
 */
public class SubmersibleIntakeState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG = SubmersibleIntakeState.class.getSimpleName();
    private final RobotState robotState;
    private final IntakeSubsystem intake;
    private int runCounter;

    /**
     * Creates a new SubmersibleIntakeState
     *
     * @param name       the name of the state
     * @param drive      the drive subsystem
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public SubmersibleIntakeState(String name, DriveSubsystem drive, IntakeSubsystem intake, RobotState robotState) {
        super(name, 3);
        this.robotState = robotState;
        this.intake = intake;
        runCounter = 0;
        addCommands(
                new IntakeTrackingAction(intake, robotState),
                new IntakePrepareToTransferAction(drive, intake, robotState::getBlockOrientation, robotState)
        );
    }

    @Override
    public void initialize() {
        runCounter++;
        super.initialize();
    }

    @Override
    public AutoState getCurrentCondition() {
        if (super.isTimeoutReached() || (IntakeSubsystem.SLIDES_MAX - intake.getCurrentSlidePositionInches() < 2.5 && robotState.isIntakeTracking())) {
            return AutoState.TIMEOUT;
        } else {
            if (robotState.getIntakeState() == IntakeState.PREPARE_TO_TRANSFER) {
                if (robotState.getBlockPosition() == RobotBlockPosition.INTAKE) {
                    return AutoState.SAMPLE_INTAKE_COMPLETE;
                } else {
                    return AutoState.SAMPLE_INTAKE_FAILED;
                }
            } else {
                return AutoState.RUNNING;
            }
        }
    }
}
