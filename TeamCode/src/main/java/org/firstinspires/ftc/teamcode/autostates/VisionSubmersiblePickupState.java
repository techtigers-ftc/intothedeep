package org.firstinspires.ftc.teamcode.autostates;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.IntakeVisionPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.LimelightLateralBoundsAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * State for using the vision system to pick a sample out from the submersible
 */
public class VisionSubmersiblePickupState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG = VisionSubmersiblePickupState.class.getSimpleName();
    private final RobotState robotState;
    private int runCounter;

    /**
     * Creates a new VisionSubmersiblePickupState
     *
     * @param name       the name of the state
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param drive      the drive subsystem
     * @param limelight  the limelight subsystem
     * @param robotState the robot state
     */
    public VisionSubmersiblePickupState(String name, IntakeSubsystem intake, DropperSubsystem dropper, DriveSubsystem drive, LimelightSubsystem limelight, RobotState robotState) {
        super(name, 5);
        this.robotState = robotState;
        runCounter = 0;
        addCommands(
                new LimelightLateralBoundsAction(limelight, -5, 1),
                new IntakeVisionPickupAction(intake, dropper, drive, robotState, () -> robotState.getRobotCurrentPose().getHeading())
        );
    }

    @Override
    public void initialize() {
        runCounter++;
        super.initialize();
    }

    @Override
    public AutoState getCurrentCondition() {
        if (super.isTimeoutReached()) {
            return AutoState.TIMEOUT;
        } else {
            if (robotState.getIntakeState() == IntakeState.READY_TO_TRANSFER) {
                if (runCounter == 1) {
                    return AutoState.SAMPLE_4_INTAKE_COMPLETE;
                } else {
                    return AutoState.SAMPLE_5_INTAKE_COMPLETE;
                }
            } else {
                return AutoState.RUNNING;
            }
        }
    }
}
