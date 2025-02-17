package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.display.view.TeleOpView;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;

import team.techtigers.base.BaseOpMode;
import team.techtigers.base.visualdisplay.AdafruitNeoPixel;
import team.techtigers.base.visualdisplay.VisualDisplaySubsystem;

@TeleOp(name="VisualDisplayTestOpMode")
public class VisualDisplayTestOpMode extends BaseOpMode {

    private RobotState robotState;
    private VisualDisplaySubsystem visualDisplaySubsystem;

    @Override
    public void initialize() {
        robotState = new RobotState(true, false);
        AdafruitNeoPixel displayDriver = hardwareMap.get(AdafruitNeoPixel.class, "visual_display");
        displayDriver.initialize(224, 3);
        VisualDisplaySubsystem visualDisplaySubsystem = new VisualDisplaySubsystem(displayDriver, new TeleOpView(robotState));
        registerSubsystems(visualDisplaySubsystem);

        robotState.setCurrentGear(DriveGears.NOT_ENGAGED);

    }
}
