package org.firstinspires.ftc.teamcode.display.view;

import org.firstinspires.ftc.teamcode.display.region.VoltageIndicatorRegion;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.DisplayView;

public class TeleView extends DisplayView {
    /**
     * Creates a new view for teleop and endgame
     */
    public TeleView(RobotState robotState) {
        super(new DisplayRegion[]{
                new VoltageIndicatorRegion(16, 6, robotState)
        });
    }
}
