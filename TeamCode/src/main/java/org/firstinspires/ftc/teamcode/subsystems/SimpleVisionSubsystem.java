package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.SampleDetectionProcessor;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.vision.VisionPortal;

import team.techtigers.base.CloseableSubsystem;

public class SimpleVisionSubsystem extends CloseableSubsystem {
    private final WebcamName camera;
    private final VisionPortal visionPortal;

    public SimpleVisionSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        camera = hardwareMap.get(WebcamName.class, "camera");
        visionPortal = new VisionPortal.Builder().setCamera(camera).setCameraResolution(new Size(640, 480)).addProcessor(new SampleDetectionProcessor()).build();

    }

    @Override
    public void close() {
        visionPortal.close();
    }
}
