package org.firstinspires.ftc.teamcode.display.region;

import org.firstinspires.ftc.teamcode.display.sprite.RectangleSprite;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.Sprite;

/**
 * A region that changes color when heading lock is enabled or disabled
 */
public class HeadingLockRegion extends DisplayRegion {
    private final RobotState robotState;
    private final RectangleSprite headingLockSprite;
    private final Sprite[] sprites;

    /**
     * Creates a new AutoStateRegion
     *
     * @param x          the x position of the region on the display
     * @param y          the y position of the region on the display
     * @param robotState the robot state
     */
    public HeadingLockRegion(int x, int y, RobotState robotState) {
        super(x, y, 2, 2);
        this.robotState = robotState;

        headingLockSprite = new RectangleSprite(0, 0, 2, 2);
        headingLockSprite.enable();
        sprites = new Sprite[]{headingLockSprite};

        headingLockSprite.setColor(Color.GREEN);
    }

    @Override
    public void update() {
        if (robotState.isHeadingLockEnabled()) {
            headingLockSprite.setColor(Color.RED);
        } else {
            headingLockSprite.setColor(Color.GREEN);
        }
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
