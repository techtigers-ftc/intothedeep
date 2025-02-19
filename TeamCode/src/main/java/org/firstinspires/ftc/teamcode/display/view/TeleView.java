package org.firstinspires.ftc.teamcode.display.view;

import org.firstinspires.ftc.teamcode.display.region.BlockDetectionStateRegion;
import org.firstinspires.ftc.teamcode.display.region.ColorPreferenceRegion;
import org.firstinspires.ftc.teamcode.display.region.CountdownTimerRegion;
import org.firstinspires.ftc.teamcode.display.region.IntakeFlashbangRegion;
import org.firstinspires.ftc.teamcode.display.region.VoltageIndicatorPartOneRegion;
import org.firstinspires.ftc.teamcode.display.region.VoltageIndicatorPartThreeRegion;
import org.firstinspires.ftc.teamcode.display.region.VoltageIndicatorPartTwoRegion;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.DisplayView;

public class TeleView extends DisplayView {
    /**
     * Creates a new view for teleop and endgame
     */
    public TeleView(RobotState robotState) {
        super(new DisplayRegion[]{
                new IntakeFlashbangRegion(0, 0, robotState),
                new CountdownTimerRegion(11, 0, robotState),
                new BlockDetectionStateRegion(21, 1, robotState),
                new ColorPreferenceRegion(3, 5, robotState),
                new IntakeFlashbangRegion(26, 0, robotState),


                new VoltageIndicatorPartOneRegion(7, 0, robotState),
                new VoltageIndicatorPartTwoRegion(10, 6, robotState),
                new VoltageIndicatorPartThreeRegion(17, 0, robotState)
        });
    }
}
