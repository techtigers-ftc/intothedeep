package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.AbsoluteBlockPosition;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;

import team.techtigers.base.CloseableSubsystem;

/**
 * A subsystem which saves a detected sample's attributes into RobotState
 */
@Config
public class LimelightSubsystem extends CloseableSubsystem {
    public static final double SLIDES_OFFSET = 7; // forward distance from center of robot to slides
    public static final double LIMELIGHT_COARSE_OFFSET = 2; // forward distance from end of slides to limelight lens in coarse
    public static final double LIMELIGHT_FINE_OFFSET = 4; // forward distance from end of slides to limelight lens in fine
    private static final double CAMERA_FINE_ANGLE = 16;
    public static double LIMELIGHT_HEIGHT = 8;
    public static double CAMERA_COARSE_ANGLE = 40;
    private final RobotState robotState;
    private final Limelight3A limelight;
    private final AbsoluteBlockPosition absoluteBlockPosition;

    private boolean isLastModeCourse;


    /**
     * Constructor for the LimelightSubsystem
     *
     * @param hardwareMap Used to get the limelight camera from list of hardware devices
     * @param robotState  Used to set limelight values in robotstate
     */
    public LimelightSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        this.robotState = robotState;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        absoluteBlockPosition = new AbsoluteBlockPosition(robotState);
        isLastModeCourse = robotState.isCoarseCameraMode();
    }

    @Override
    public void init() {
        limelight.setPollRateHz(50);
        limelight.start();
        limelight.pipelineSwitch(3);
    }

    @Override
    public void periodic() {
        double blue = robotState.isBlue() ? 1 : 0;
        double red = robotState.isBlue() ? 0 : 1;
        double yellow = 0;
        double coarseCamera = robotState.isCoarseCameraMode() ? 0 : 1;

        if (robotState.getBlockColorPreference() == BlockColorPreference.YELLOW) {
            yellow = 1;
            blue = 0;
            red = 0;
        } else if (robotState.getBlockColorPreference() == BlockColorPreference.ANY) {
            yellow = 1;
        }

        limelight.updatePythonInputs(yellow, red, blue, coarseCamera, robotState.getIntakeSlidePosition(), IntakeSubsystem.SLIDES_MAX, 0, 0);

        LLResult result = limelight.getLatestResult();

        if (result != null) {
            double[] results = result.getPythonOutput();
            if ((results.length < 4) || (results[1] == 0 && results[2] == 0 && results[3] == 0)) {
                robotState.setFineBlockDetectionState(BlockDetectionState.NOT_DETECTED);
                robotState.setCoarseBlockDetectionState(BlockDetectionState.NOT_DETECTED);
            } else {
                if (!robotState.isCoarseCameraMode()) {
                    double forwardFine =
                            LIMELIGHT_HEIGHT * Math.tan(Math.toRadians(CAMERA_FINE_ANGLE + result.getTy()));
                    double lateralFine =
                            Math.hypot(forwardFine, LIMELIGHT_HEIGHT) * Math.tan(Math.toRadians(result.getTx()));
                    double orientation = (results[3] + 180) % 180;

                    robotState.setCoarseBlockDetectionState(BlockDetectionState.NOT_DETECTED);
                    robotState.setFineBlockDetectionState(BlockDetectionState.DETECTED);
                    robotState.setBlockForwardFine(forwardFine);
                    robotState.setBlockLateralFine(lateralFine);
                    robotState.setBlockOrientation(orientation);

                    // Saving the block's relative position whenever a block is seen
                    absoluteBlockPosition.updatePosition(
                            robotState.getRobotCurrentPose(),
                            lateralFine,
                            forwardFine + SLIDES_OFFSET + LIMELIGHT_FINE_OFFSET + robotState.getIntakeSlidePosition(),
                            robotState.getBlockOrientation()
                    );

                    robotState.setAbsoluteBlockPosition(absoluteBlockPosition.getAbsoluteBlockPosition());
                } else {
                    double forwardCoarse = LIMELIGHT_HEIGHT * Math.tan(Math.toRadians(CAMERA_COARSE_ANGLE + result.getTy()));
                    double lateralCoarse = forwardCoarse * Math.tan(Math.toRadians(result.getTx()));

                    robotState.setCoarseBlockDetectionState(BlockDetectionState.DETECTED);
                    robotState.setFineBlockDetectionState(BlockDetectionState.NOT_DETECTED);
                    robotState.setBlockForwardCoarse(forwardCoarse);
                    robotState.setBlockLateralCoarse(lateralCoarse);

                    // Saving the block's relative position whenever a block is seen
                    absoluteBlockPosition.updatePosition(
                            robotState.getRobotCurrentPose(),
                            lateralCoarse,
                            forwardCoarse + SLIDES_OFFSET + LIMELIGHT_COARSE_OFFSET + robotState.getIntakeSlidePosition(),
                            robotState.getBlockOrientation()
                    );

                    robotState.setAbsoluteBlockPosition(absoluteBlockPosition.getAbsoluteBlockPosition());
                }
            }
        }

        if (isLastModeCourse != robotState.isCoarseCameraMode()) {
            isLastModeCourse = robotState.isCoarseCameraMode();
            absoluteBlockPosition.resetBlockDetection();
        }
        robotState.setBlockDetected(absoluteBlockPosition.isBlockDetected());
    }

    @Override
    public void close() {
        limelight.close();
    }
}