package org.firstinspires.ftc.teamcode.display.view;

import org.firstinspires.ftc.teamcode.display.region.BlockDetectionStateRegion;
import org.firstinspires.ftc.teamcode.display.region.ColorPreferenceRegion;
import org.firstinspires.ftc.teamcode.display.region.CountdownTimerRegion;
import org.firstinspires.ftc.teamcode.display.region.DriveGearsRegion;
import org.firstinspires.ftc.teamcode.display.region.IntakeFlashbangRegion;
import org.firstinspires.ftc.teamcode.display.region.SwitchingTransmissionStatusRegion;
import org.firstinspires.ftc.teamcode.display.region.VoltageIndicatorRegion;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.DisplayView;

/**
 * A class for the end game view on the LEDs
 */
public class EndgameView extends DisplayView {
    public EndgameView(RobotState robotState) {
        super(new DisplayRegion[]{
                new IntakeFlashbangRegion(0, 0, robotState),
                new CountdownTimerRegion(2, 0, robotState),
                new DriveGearsRegion(2, 5, robotState),
                new ColorPreferenceRegion(5, 5, robotState),
                new BlockDetectionStateRegion(8, 0, robotState),
                new SwitchingTransmissionStatusRegion(8, 5, robotState),
                new VoltageIndicatorRegion(11,0, robotState),
                new IntakeFlashbangRegion(22, 0, robotState)
        });
    }
}