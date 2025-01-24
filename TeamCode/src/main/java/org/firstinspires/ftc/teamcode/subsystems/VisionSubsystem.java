package org.firstinspires.ftc.teamcode.subsystems;

import static android.os.SystemClock.sleep;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.cv.SampleDetectionProcessor;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

import team.techtigers.base.CloseableSubsystem;

/**
 * A subsystem for using the intake camera
 */
public class VisionSubsystem extends CloseableSubsystem {
    public static final double INTAKE_CAMERA_OFFSET = 2;
    public static int EXPOSURE = 16;
    public static int GAIN = 0;
    private final WebcamName camera;
    private final VisionPortal visionPortal;
    private final RobotState robotState;

    /**
     * Construct a VisionSubsystem
     *
     * @param hardwareMap The HardwareMap
     * @param robotState  The RobotState
     */
    public VisionSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        this.robotState = robotState;
        camera = hardwareMap.get(WebcamName.class, "camera");
        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .setCameraResolution(new Size(SampleDetectionProcessor.WIDTH_RESOLUTION, SampleDetectionProcessor.HEIGHT_RESOLUTION))
                .addProcessor(new SampleDetectionProcessor(robotState))
                .build();
    }

    @Override
    public void init() {
        visionPortal.stopLiveView();
        visionPortal.getCameraState();
//        setExposure();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(100);
        }
    }

    private void setExposure() {
        ExposureControl exposureControl;
        // Running this to catch any IllegalStateExceptions thrown by
        // starting the opmode too fast
        try {
            exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        } catch (IllegalStateException e) {
            sleep(100);
            setExposure();
            return;
        }
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposureControl.setExposure(EXPOSURE, TimeUnit.MILLISECONDS);
        sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        sleep(20);
        gainControl.setGain(GAIN);

    }

    @Override
    public void close() {
        visionPortal.close();
    }
}
