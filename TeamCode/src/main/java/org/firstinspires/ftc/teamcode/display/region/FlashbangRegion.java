package org.firstinspires.ftc.teamcode.display.region;

import org.firstinspires.ftc.teamcode.display.sprite.RectangleSprite;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColor;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.Sprite;

/**
 * A region that flashes the sprite when the intake picks up a block
 */
public class FlashbangRegion extends DisplayRegion {
    private final RobotState robotState;
    private final RectangleSprite rectangleSprite;
    private final Sprite[] sprites;

    /**
     * Creates a new IntakeFlashbangRegion
     *
     * @param x          the x position of the region on the display
     * @param y          the y position of the region on the display
     * @param robotState the robot state
     */
    public FlashbangRegion(int x, int y, RobotState robotState) {
        super(x, y, 2, 8);
        this.robotState = robotState;

        rectangleSprite = new RectangleSprite(0, 0, 2, 8);
        rectangleSprite.enable();

        this.sprites = new Sprite[]{rectangleSprite};
    }


    @Override
    public void update() {
        if (robotState.getIntakeBlockColor() == BlockColor.NONE) {
            rectangleSprite.setColor(Color.RED);
        } else {
            rectangleSprite.setColor(Color.GREEN);
        }
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
