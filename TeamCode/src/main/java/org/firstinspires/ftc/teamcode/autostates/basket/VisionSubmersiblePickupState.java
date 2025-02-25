package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTrackingAction;
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
public class VisionSubmersiblePickupState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG = VisionSubmersiblePickupState.class.getSimpleName();
    private final RobotState robotState;
    private final IntakeSubsystem intake;
    private int runCounter;

    /**
     * Creates a new VisionSubmersiblePickupState
     *
     * @param name       the name of the state
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param drive      the drive subsystem
     * @param robotState the robot state
     */
    public VisionSubmersiblePickupState(String name, IntakeSubsystem intake, DropperSubsystem dropper, DriveSubsystem drive, RobotState robotState) {
        super(name, 6);
        this.robotState = robotState;
        this.intake = intake;
        runCounter = 0;
        addCommands(
                new IntakeTrackingAction(intake, 1, robotState),
                new WaitCommand(100),
                new IntakePrepareToTransferAction(drive, intake, dropper, robotState)
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
            if (robotState.getIntakeState() == IntakeState.PREPARE_TO_TRANSFER) {
//                if (!intake.isBlockInIntake()) {
//                    return AutoState.NO_TIME;
//                }
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
