package org.firstinspires.ftc.teamcode.display.region;

import org.firstinspires.ftc.teamcode.display.sprite.RectangleSprite;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.Sprite;

/**
 * A region which shows an indicator that shows the current status of the battery's voltage
 */
public class VoltageIndicatorPartOneRegion extends DisplayRegion {
    private final RobotState robotState;
    private final RectangleSprite L1Sprite;
    private final RectangleSprite L2Sprite;
    private final RectangleSprite L3Sprite;
    private final RectangleSprite L4Sprite;
    private final RectangleSprite L5Sprite;
    private final RectangleSprite L6Sprite;

    private final static double MIN_VOLTAGE_THRESHOLD = 8;
    private final static double MAX_VOLTAGE_THRESHOLD = 9.83;

    private final static double STEP = (MAX_VOLTAGE_THRESHOLD - MIN_VOLTAGE_THRESHOLD) / 6;

    private final static double L1_THRESHOLD = MIN_VOLTAGE_THRESHOLD + STEP;
    private final static double L2_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 2 * STEP;
    private final static double L3_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 3 * STEP;
    private final static double L4_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 4 * STEP;
    private final static double L5_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 5 * STEP;
    private final static double L6_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 6 * STEP;

    private final Sprite[] sprites;

    /**
     * Creates a new VoltageIndicatorRegionPartOne
     *
     * @param x          the x position of the region
     * @param y          the y position of the region
     * @param robotState the robot state
     */
    public VoltageIndicatorPartOneRegion(int x, int y, RobotState robotState) {
        super(x, y, 4, 6);
        this.robotState = robotState;
        L1Sprite = new RectangleSprite(0, 0, 2, 1);
        L2Sprite = new RectangleSprite(0, 1, 2, 1);
        L3Sprite = new RectangleSprite(0, 2, 2, 1);
        L4Sprite = new RectangleSprite(1, 3, 2, 1);
        L5Sprite = new RectangleSprite(1, 4, 2, 1);
        L6Sprite = new RectangleSprite(2, 5, 2, 1);


        sprites = new Sprite[]{L1Sprite, L2Sprite, L3Sprite, L4Sprite,
                L5Sprite, L6Sprite
        };

        for (int i = 0; i < 4; i++) {
            sprites[i].setColor(Color.RED);
        }
        for (int i = 0; i < 2; i++) {
            sprites[i + 4].setColor(Color.YELLOW);
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
        } else {
            enableAllSprites();
        }
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
