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
    private final RectangleSprite rectangleSpriteOne;
    private final RectangleSprite rectangleSpriteTwo;
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
        rectangleSpriteOne = new RectangleSprite(0, 0, 2, 2);
        rectangleSpriteTwo = new RectangleSprite(0, 0, 2, 2);
        rectangleSpriteOne.enable();
        rectangleSpriteTwo.enable();

        sprites = new Sprite[]{rectangleSpriteOne, rectangleSpriteTwo};
    }


    @Override
    public void update() {
        if (robotState.getBlockColorPreference() == BlockColorPreference.YELLOW) {
            rectangleSpriteOne.setColor(Color.YELLOW);
            rectangleSpriteTwo.setColor(Color.YELLOW);
        } else if (robotState.getBlockColorPreference() == BlockColorPreference.ALLIANCE) {
            if (robotState.isBlue()) {
                rectangleSpriteOne.setColor(Color.BLUE);
                rectangleSpriteTwo.setColor(Color.BLUE);
            } else {
                rectangleSpriteOne.setColor(Color.RED);
                rectangleSpriteTwo.setColor(Color.RED);
            }
        } else {
            if (robotState.isBlue()) {
                rectangleSpriteOne.setColor(Color.BLUE);
                rectangleSpriteTwo.setColor(Color.YELLOW);
            } else {
                rectangleSpriteOne.setColor(Color.RED);
                rectangleSpriteTwo.setColor(Color.YELLOW);
            }
        }
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
