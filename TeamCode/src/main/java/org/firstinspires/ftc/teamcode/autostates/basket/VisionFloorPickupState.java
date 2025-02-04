package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.ChangeBlockColorPreferenceCommand;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.IntakeVisionPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.LimelightLateralBoundsAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * A state to intake a sample
 */
public class VisionFloorPickupState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            VisionFloorPickupState.class.getSimpleName();
    private final RobotState robotState;
    private int runCounter;

    /**
     * Constructor for the VisionFloorPickupState
     *
     * @param name       The name of the state
     * @param intake     The intake subsystem
     * @param dropper    The dropper subsystem
     * @param drive      the drive subsystem
     * @param limelight  the limelight subsystem
     * @param robotState The robot state
     */
    public VisionFloorPickupState(String name, IntakeSubsystem intake,
                                  DropperSubsystem dropper,
                                  DriveSubsystem drive,
                                  LimelightSubsystem limelight,
                                  RobotState robotState) {
        super(name, 5);
        this.robotState = robotState;
        runCounter = 0;
        addCommands(
                new LimelightLateralBoundsAction(limelight, -3, 3),
                new WaitCommand(250),
                new IntakeVisionPickupAction(intake, dropper, drive, robotState,
                        () -> robotState.getRobotCurrentPose().getHeading(),
                        () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading()),
                        null));
    }

    @Override
    public void initialize() {
        runCounter++;
        super.initialize();
        robotState.setBlockColorPreference(BlockColorPreference.YELLOW);
    }

    @Override
    public void execute() {
        super.execute();
        RobotLog.dd(LOG_TAG, "forward limelight distance: %f", robotState.getBlockForwardCoarse());
    }

    /**
     * Get the current condition of the robot
     *
     * @return the current condition of the robot using the AutoState enum
     */
    @Override
    public AutoState getCurrentCondition() {
        if (super.isTimeoutReached()) {
            return AutoState.TIMEOUT;
        } else {
            if (robotState.getIntakeState() == IntakeState.READY_TO_TRANSFER) {
                if (runCounter == 1) {
                    return AutoState.SAMPLE_1_INTAKE_COMPLETE;
                } else if (runCounter == 2) {
                    return AutoState.SAMPLE_2_INTAKE_COMPLETE;
                } else {
                    return AutoState.SAMPLE_3_INTAKE_COMPLETE;
                }
            } else {
                return AutoState.RUNNING;
            }
        }
    }
}
