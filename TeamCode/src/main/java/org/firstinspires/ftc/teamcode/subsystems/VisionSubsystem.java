package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.SampleDetectionProcessor;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.vision.VisionPortal;

import team.techtigers.base.CloseableSubsystem;

public class VisionSubsystem extends CloseableSubsystem {
    private final RobotState robotState;
    private final WebcamName webcam;
    private final SampleDetectionProcessor processor;
    private final VisionPortal visionPortal;


    /**
     * Creates a new VisionSubsystem.
     *
     * @param hardwareMap The hardware map to get the camera from
     * @param robotState  The robot State to update values to
     */
    public VisionSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        super();
        this.robotState = robotState;
        webcam = hardwareMap.get(WebcamName.class, "camera");
        this.processor = new SampleDetectionProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessors(processor)
                .build();
    }

    @Override
    public void periodic() {
        double[] foundSample = processor.getFoundSample();
//        RobotLog.dd(tag, "is block detected: %f", processor.isBlockDetected());
        if (processor.isBlockDetected()) {
            robotState.setBlockLateralFine(foundSample[0]);
            robotState.setBlockForwardFine(foundSample[1]);
//            robotState.setBlockOrientation(foundSample[2]);
            RobotLog.dd(tag, "Block Lateral: %f, Block Forward: %f, Block Orientation: %f", foundSample[0], foundSample[1], foundSample[2]);
        }
    }

    @Override
    public void close() {
        visionPortal.close();
    }
}