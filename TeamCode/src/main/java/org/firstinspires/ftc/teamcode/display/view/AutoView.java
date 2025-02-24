package org.firstinspires.ftc.teamcode.display.view;

import org.firstinspires.ftc.teamcode.display.region.AutoStateRegion;
import org.firstinspires.ftc.teamcode.display.region.ColorPreferenceRegion;
import org.firstinspires.ftc.teamcode.display.region.CountdownTimerRegion;
import org.firstinspires.ftc.teamcode.display.region.FlashbangRegion;
import org.firstinspires.ftc.teamcode.display.region.VisionStreamRegion;
import org.firstinspires.ftc.teamcode.display.region.VoltageIndicatorRegion;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.DisplayView;

/**
 * A view for autonomous stage of the game
 */
public class AutoView extends DisplayView {
    /**
     * Creates a new view for autonomous
     * @param robotState the robot state
     */
    public AutoView(RobotState robotState) {
        super(new DisplayRegion[]{
                new AutoStateRegion(0, 0, robotState),
                new VisionStreamRegion(2, 0, robotState),
                new ColorPreferenceRegion(15, 0, robotState),
                new VoltageIndicatorRegion(15, 6, robotState),
                new CountdownTimerRegion(18, 0, robotState),
                new AutoStateRegion(26, 0, robotState)
        });
    }
}
