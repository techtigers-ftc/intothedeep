package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.display.view.TeleView;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import team.techtigers.base.BaseOpMode;
import team.techtigers.base.visualdisplay.AdafruitNeoPixel;
import team.techtigers.base.visualdisplay.VisualDisplaySubsystem;

@Config
@TeleOp(name = "VisualDisplayTestOpMode")
public class VisualDisplayTestOpMode extends BaseOpMode {

    public static double voltage = 14;
    public static double lateralFine = 0;
    public static double forwardFine = 0;
    public static double orientation = 0;
    public static boolean blockInRobot = false;
    public static boolean blockDetected = true;
    private ElapsedTime timer;

    private RobotState robotState;
    private VisualDisplaySubsystem visualDisplaySubsystem;

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);

        robotState = new RobotState(false, false);
        AdafruitNeoPixel displayDriver = hardwareMap.get(AdafruitNeoPixel.class, "visual_display");
        displayDriver.initialize(224, 3);
        VisualDisplaySubsystem visualDisplaySubsystem = new VisualDisplaySubsystem(displayDriver, new TeleView(robotState));
        registerSubsystems(visualDisplaySubsystem);
        timer = new ElapsedTime();
        timer.reset();

        robotState.setCurrentGear(DriveGears.NOT_ENGAGED);
        telemetry.addData("Cycle time", timer.milliseconds());
    }

    @Override
    public void update() {
        robotState.setVoltage(voltage);
        robotState.setBlockLateralFine(lateralFine);
        robotState.setBlockForwardFine(forwardFine);
        robotState.setBlockOrientation(orientation);
        if (blockDetected) robotState.setFineBlockDetectionState(BlockDetectionState.DETECTED);
        else robotState.setFineBlockDetectionState(BlockDetectionState.NOT_DETECTED);
        if (blockInRobot) robotState.setBlockPosition(RobotBlockPosition.INTAKE);
        else robotState.setBlockPosition(RobotBlockPosition.NONE);

        telemetry.addData("Cycle time", timer.milliseconds());
        timer.reset();
    }
}
