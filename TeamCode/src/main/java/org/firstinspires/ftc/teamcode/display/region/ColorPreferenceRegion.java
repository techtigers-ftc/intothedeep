package org.firstinspires.ftc.teamcode.display.region;

import org.firstinspires.ftc.teamcode.display.sprite.RectangleSprite;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.Sprite;

/**
 * A region which shows the color preference of the robot
 */
public class ColorPreferenceRegion extends DisplayRegion {
    private final RobotState robotState;
    private final RectangleSprite rectangleSprite;
    private final Sprite[] sprites;

    /**
     * Creates a new ColorPreferenceRegion
     *
     * @param x          the x position of the region
     * @param y          the y position of the region
     * @param robotState the robot state
     */
    public ColorPreferenceRegion(int x, int y, RobotState robotState) {
        super(x, y, 4, 2);
        this.robotState = robotState;
        rectangleSprite = new RectangleSprite(0, 0, 4, 2);
        rectangleSprite.enable();

        sprites = new Sprite[]{rectangleSprite};
    }


    @Override
    public void update() {
        if (robotState.getBlockColorPreference() == BlockColorPreference.YELLOW) {
            this.rectangleSprite.setColor(Color.YELLOW);
        } else if (robotState.getBlockColorPreference() == BlockColorPreference.ALLIANCE) {
            if (robotState.isBlue()) {
                this.rectangleSprite.setColor(Color.BLUE);
            } else {
                this.rectangleSprite.setColor(Color.RED);
            }
        } else {
            this.rectangleSprite.setColor(Color.GREEN);
        }
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
