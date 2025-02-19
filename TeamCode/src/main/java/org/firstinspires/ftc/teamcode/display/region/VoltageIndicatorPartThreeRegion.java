package org.firstinspires.ftc.teamcode.display.region;

import org.firstinspires.ftc.teamcode.display.sprite.RectangleSprite;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.Sprite;

/**
 * A region which shows an indicator that shows the current status of the battery's voltage
 */
public class VoltageIndicatorPartThreeRegion extends DisplayRegion {
    private final RobotState robotState;
    private final RectangleSprite L13Sprite;
    private final RectangleSprite L14Sprite;
    private final RectangleSprite L15Sprite;
    private final RectangleSprite L16Sprite;
    private final RectangleSprite L17Sprite;
    private final RectangleSprite L18Sprite;

    private final static double MIN_VOLTAGE_THRESHOLD = 8;
    private final static double MAX_VOLTAGE_THRESHOLD = 13.5;

    private final static double STEP = (MAX_VOLTAGE_THRESHOLD - MIN_VOLTAGE_THRESHOLD) / 18;

    private final static double L13_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 13 * STEP;
    private final static double L14_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 14 * STEP;
    private final static double L15_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 15 * STEP;
    private final static double L16_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 16 * STEP;
    private final static double L17_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 17 * STEP;
    private final static double L18_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 18 * STEP;

    private final Sprite[] sprites;

    /**
     * Creates a new VoltageIndicatorRegionPartThree
     *
     * @param x          the x position of the region
     * @param y          the y position of the region
     * @param robotState the robot state
     */
    public VoltageIndicatorPartThreeRegion(int x, int y, RobotState robotState) {
        super(x, y, 4, 6);
        this.robotState = robotState;
        L13Sprite = new RectangleSprite(0, 5, 2, 1);
        L14Sprite = new RectangleSprite(1, 4, 2, 1);
        L15Sprite = new RectangleSprite(1, 3, 2, 1);
        L16Sprite = new RectangleSprite(2, 2, 2, 1);
        L17Sprite = new RectangleSprite(2, 1, 2, 1);
        L18Sprite = new RectangleSprite(2, 0, 2, 1);


        sprites = new Sprite[]{L13Sprite, L14Sprite, L15Sprite, L16Sprite, L17Sprite, L18Sprite};

        for (int i = 0; i < 6; i++) {
            sprites[i].setColor(Color.GREEN);
        }
    }


    @Override
    public void update() {
        double currentVoltage = robotState.getVoltage();
        if (currentVoltage < L13_THRESHOLD) {
            disableAllButSelectedSprites(0);
        } else if (currentVoltage < L14_THRESHOLD) {
            disableAllButSelectedSprites(0, 1);
        } else if (currentVoltage < L15_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2);
        } else if (currentVoltage < L16_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3);
        } else if (currentVoltage < L17_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4);
        } else if (currentVoltage < L18_THRESHOLD) {
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
