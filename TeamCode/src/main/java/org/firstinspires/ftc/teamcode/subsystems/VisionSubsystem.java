package org.firstinspires.ftc.teamcode.subsystems;

import static android.os.SystemClock.sleep;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.SampleDetectionProcessor;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

import java.util.concurrent.TimeUnit;

import team.techtigers.base.CloseableSubsystem;

/**
 * A subsystem for using the intake camera
 */
public class VisionSubsystem extends CloseableSubsystem {
    private final WebcamName camera;
    private final VisionPortal visionPortal;
    public static int EXPOSURE = 16;
    public static int GAIN = 0;

    /**
     * Construct a VisionSubsystem
     * @param hardwareMap The HardwareMap
     * @param robotState The RobotState
     */
    public VisionSubsystem(HardwareMap hardwareMap, RobotState robotState) {
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
        setExposure();
    }

    private void setExposure() {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
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
