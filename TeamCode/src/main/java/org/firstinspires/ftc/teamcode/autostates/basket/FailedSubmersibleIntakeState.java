package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.TeleHoldPointAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTrackingAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * State for using the vision system to pick a sample out from the submersible if the first attempt fails
 */
public class FailedSubmersibleIntakeState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG = FailedSubmersibleIntakeState.class.getSimpleName();
    private final RobotState robotState;
    private final IntakeSubsystem intake;

    /**
     * Creates a new FailedSubmersibleIntakeState
     *
     * @param name       the name of the state
     * @param drive      the drive subsystem
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public FailedSubmersibleIntakeState(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        super(name, 6);
        this.robotState = robotState;
        this.intake = intake;
        addCommands(
                new ParallelCommandGroup(
                        new IntakeSlidesAbsoluteAction(intake, () -> 3, 0.75),
                        new TeleHoldPointAction(
                                drive, robotState,
                                () -> robotState.getRobotCurrentPose().getX(),
                                () -> robotState.getRobotCurrentPose().getY() + 3,
                                () -> robotState.getRobotCurrentPose().getHeading(), 0.5, Math.toRadians(2)
                        )
                ),
                new IntakeTrackingAction(intake, robotState),
                new WaitUntilCommand(() -> robotState.getFineBlockDetectionState() == BlockDetectionState.DETECTED),
                new IntakePrepareToTransferAction(drive, intake, dropper, robotState::getBlockOrientation, robotState)
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (super.isTimeoutReached()) {
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
