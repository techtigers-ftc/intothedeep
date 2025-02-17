package org.firstinspires.ftc.teamcode.display.view;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.display.region.BlockDetectionStateRegion;
import org.firstinspires.ftc.teamcode.display.region.ColorPreferenceRegion;
import org.firstinspires.ftc.teamcode.display.region.CountdownTimerRegion;
import org.firstinspires.ftc.teamcode.display.region.DriveGearsRegion;
import org.firstinspires.ftc.teamcode.display.region.IntakeFlashbangRegion;
import org.firstinspires.ftc.teamcode.display.sprite.RectangleSprite;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.DisplayView;
import team.techtigers.core.display.Sprite;

/**
 * A class for the tele op view on the visual display
 */
public class TeleOpView extends DisplayView {
    public TeleOpView(RobotState robotState) {
        super(new DisplayRegion[]{
//                new IntakeFlashbangRegion(0, 0, robotState),
                new CountdownTimerRegion(2, 0, robotState),
                new DriveGearsRegion(2, 5, robotState),
                new ColorPreferenceRegion(5, 5, robotState),
                new BlockDetectionStateRegion(8, 0, robotState),
//                new IntakeFlashbangRegion(22, 0, robotState)
//                new DummyRegion(0, 0)
        });
    }
}
class DummyRegion extends DisplayRegion {
    private final RectangleSprite rectangleSprite;
    private final Sprite[] sprites;
    private final ElapsedTime refreshTimer = new ElapsedTime();

    /**
     * Creates a new ColorPreferenceRegion
     *
     * @param x          the x position of the region
     * @param y          the y position of the region
     */
    public DummyRegion(int x, int y) {
        super(x, y, 28, 8);
        rectangleSprite = new RectangleSprite(0, 0, 28, 8);
        rectangleSprite.enable();
        refreshTimer.reset();

        sprites = new Sprite[]{rectangleSprite};
    }

    @Override
    public void update() {
        if(refreshTimer.milliseconds() < 1000) {
            return;
        }
        if (rectangleSprite.getColor() == Color.GREEN) {
            rectangleSprite.setColor(Color.RED);
        } else if (rectangleSprite.getColor() == Color.RED) {
            rectangleSprite.setColor(Color.BLUE);
        } else {
            rectangleSprite.setColor(Color.GREEN);
        }

        refreshTimer.reset();
        RobotLog.dd("DummyRegion", "Color = %s", rectangleSprite.getColor());
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
