package org.firstinspires.ftc.teamcode.display.region.unused;

import org.firstinspires.ftc.teamcode.display.sprite.RectangleSprite;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.Sprite;

/**
 * A region which shows the status of the switching transmission
 */
public class SwitchingTransmissionStatusRegion extends DisplayRegion {
    private final RobotState robotState;
    private final RectangleSprite rectangleSprite;
    private final Sprite[] sprites;

    /**
     * Creates a new SwitchingTransmissionStatusRegion
     *
     * @param x          the x position of the region
     * @param y          the y position of the region
     * @param robotState the robot state
     */
    public SwitchingTransmissionStatusRegion(int x, int y, RobotState robotState) {
        super(x, y, 2, 2);
        this.robotState = robotState;
        rectangleSprite = new RectangleSprite(0, 0, 2, 2);
        rectangleSprite.enable();
        rectangleSprite.setColor(Color.RED);

        sprites = new Sprite[]{rectangleSprite};
    }


    @Override
    public void update() {
        if (robotState.getIsAscending()) {
            rectangleSprite.setColor(Color.GREEN);
        }
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
