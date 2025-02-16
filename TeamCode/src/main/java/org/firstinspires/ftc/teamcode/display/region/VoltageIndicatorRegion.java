package org.firstinspires.ftc.teamcode.display.region;

import org.firstinspires.ftc.teamcode.display.sprite.RectangleSprite;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.Sprite;

/**
 * A region which shows an indicator that shows the current status of the battery's voltage
 */
public class VoltageIndicatorRegion extends DisplayRegion {
    private final RobotState robotState;
    private final RectangleSprite L1Sprite;
    private final RectangleSprite L2Sprite;
    private final RectangleSprite L3Sprite;
    private final RectangleSprite L4Sprite;
    private final RectangleSprite L5Sprite;
    private final RectangleSprite L6Sprite;
    private final RectangleSprite L7Sprite;
    private final RectangleSprite L8Sprite;

    private final static double L1_THRESHOLD = 9.5;
    private final static double L2_THRESHOLD = 10;
    private final static double L3_THRESHOLD = 10.5;
    private final static double L4_THRESHOLD = 11.75;
    private final static double L5_THRESHOLD = 12.25;
    private final static double L6_THRESHOLD = 12.75;
    private final static double L7_THRESHOLD = 13;
    private final static double L8_THRESHOLD = 13.75;

    private final Sprite[] sprites;

    /**
     * Creates a new VoltageIndicatorRegion
     *
     * @param x          the x position of the region
     * @param y          the y position of the region
     * @param robotState the robot state
     */
    public VoltageIndicatorRegion(int x, int y, RobotState robotState) {
        super(x, y, 2, 8);
        this.robotState = robotState;
        L1Sprite = new RectangleSprite(0, 0, 2, 1);
        L2Sprite = new RectangleSprite(0, 1, 2, 1);
        L3Sprite = new RectangleSprite(0, 2, 2, 1);
        L4Sprite = new RectangleSprite(0, 3, 2, 1);
        L5Sprite = new RectangleSprite(0, 4, 2, 1);
        L6Sprite = new RectangleSprite(0, 5, 2, 1);
        L7Sprite = new RectangleSprite(0, 6, 2, 1);
        L8Sprite = new RectangleSprite(0, 7, 2, 1);

        sprites = new Sprite[]{L1Sprite, L2Sprite, L3Sprite, L4Sprite,
                L5Sprite, L6Sprite, L7Sprite, L8Sprite
        };

        for (int i = 0; i < 3; i++) {
            sprites[i].setColor(Color.RED);
        }
        for (int i = 0; i < 2; i++) {
            sprites[i + 3].setColor(Color.ORANGE);
        }
        for (int i = 0; i < 3; i++) {
            sprites[i + 5].setColor(Color.GREEN);
        }
    }


    @Override
    public void update() {
        double currentVoltage = robotState.getVoltage();
        if (currentVoltage < L1_THRESHOLD) {
            disableAllButSelectedSprites(0);
        } else if (currentVoltage < L2_THRESHOLD) {
            disableAllButSelectedSprites(0, 1);
        } else if (currentVoltage < L3_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2);
        } else if (currentVoltage < L4_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3);
        } else if (currentVoltage < L5_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4);
        } else if (currentVoltage < L6_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4, 5);
        } else if (currentVoltage < L7_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4, 5, 6);
        } else if (currentVoltage < L8_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4, 5, 6, 7);
        } else {
            enableAllSprites();
        }
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
