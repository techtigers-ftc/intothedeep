package org.firstinspires.ftc.teamcode.display.region;

import org.firstinspires.ftc.teamcode.display.sprite.PlusSprite;
import org.firstinspires.ftc.teamcode.display.sprite.XSprite;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.Sprite;

/**
 * A region that displays the current drive gear of the robot
 */
public class DriveGearsRegion extends DisplayRegion {
    private final RobotState robotState;
    private final XSprite slowGearSprite;
    private final PlusSprite fastGearSprite;
    private final Sprite[] sprites;

    /**
     * Creates a new DriveGearsRegion
     *
     * @param x          the x position of the region on the display
     * @param y          the y position of the region on the display
     * @param robotState the robot state
     */
    public DriveGearsRegion(int x, int y, RobotState robotState) {
        super(x, y, 3, 3);
        this.robotState = robotState;

        slowGearSprite = new XSprite(0, 0, 3, 3);
        fastGearSprite = new PlusSprite(0, 0);

        slowGearSprite.setColor(Color.RED);
        fastGearSprite.setColor(Color.GREEN);

        sprites = new Sprite[]{slowGearSprite, fastGearSprite};
    }


    @Override
    public void update() {
        if (robotState.getCurrentGear() == DriveGears.ENGAGED) {
            slowGearSprite.disable();
            fastGearSprite.enable();
        } else {
            slowGearSprite.enable();
            fastGearSprite.disable();
        }
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
