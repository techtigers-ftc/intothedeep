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
    private final RectangleSprite L9Sprite;
    private final RectangleSprite L10Sprite;
    private final RectangleSprite L11Sprite;
    private final RectangleSprite L12Sprite;
    private final RectangleSprite L13Sprite;
    private final RectangleSprite L14Sprite;
    private final RectangleSprite L15Sprite;
    private final RectangleSprite L16Sprite;
    private final RectangleSprite L17Sprite;
    private final RectangleSprite L18Sprite;

    private final static double MIN_VOLTAGE_THRESHOLD = 8;
    private final static double MAX_VOLTAGE_THRESHOLD = 13.5;

    private final static double STEP = (MAX_VOLTAGE_THRESHOLD - MIN_VOLTAGE_THRESHOLD) / 18;

    private final static double L1_THRESHOLD = MIN_VOLTAGE_THRESHOLD + STEP;
    private final static double L2_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 2 * STEP;
    private final static double L3_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 3 * STEP;
    private final static double L4_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 4 * STEP;
    private final static double L5_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 5 * STEP;
    private final static double L6_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 6 * STEP;
    private final static double L7_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 7 * STEP;
    private final static double L8_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 8 * STEP;
    private final static double L9_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 9 * STEP;
    private final static double L10_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 10 * STEP;
    private final static double L11_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 11 * STEP;
    private final static double L12_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 12 * STEP;
    private final static double L13_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 13 * STEP;
    private final static double L14_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 14 * STEP;
    private final static double L15_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 15 * STEP;
    private final static double L16_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 16 * STEP;
    private final static double L17_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 17 * STEP;
    private final static double L18_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 18 * STEP;

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
        L3Sprite = new RectangleSprite(1, 2, 2, 1);
        L4Sprite = new RectangleSprite(1, 3, 2, 1);
        L5Sprite = new RectangleSprite(2, 4, 2, 1);
        L6Sprite = new RectangleSprite(3, 5, 2, 1);
        L7Sprite = new RectangleSprite(4, 6, 2, 1);
        L8Sprite = new RectangleSprite(6, 6, 1, 2);
        L9Sprite = new RectangleSprite(7, 6, 1, 2);
        L10Sprite = new RectangleSprite(8, 6, 1, 2);
        L11Sprite = new RectangleSprite(9, 6, 1, 2);
        L12Sprite = new RectangleSprite(10, 6, 2, 1);
        L13Sprite = new RectangleSprite(11, 5, 2, 1);
        L14Sprite = new RectangleSprite(12, 4, 2, 1);
        L15Sprite = new RectangleSprite(13, 3, 2, 1);
        L16Sprite = new RectangleSprite(13, 2, 2, 1);
        L17Sprite = new RectangleSprite(14, 1, 2, 1);
        L18Sprite = new RectangleSprite(14, 0, 2, 1);


        sprites = new Sprite[]{L1Sprite, L2Sprite, L3Sprite, L4Sprite,
                L5Sprite, L6Sprite, L7Sprite, L8Sprite, L9Sprite, L10Sprite, L11Sprite, L12Sprite,
                L13Sprite, L14Sprite, L15Sprite, L16Sprite, L17Sprite, L18Sprite
        };

        for (int i = 0; i < 4; i++) {
            sprites[i].setColor(Color.RED);
        }
        for (int i = 0; i < 7; i++) {
            sprites[i + 4].setColor(Color.YELLOW);
        }
        for (int i = 0; i < 7; i++) {
            sprites[i + 7].setColor(Color.GREEN);
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
        } else if (currentVoltage < L9_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4, 5, 6, 7, 8);
        } else if (currentVoltage < L10_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4, 5, 6, 7, 8, 9);
        } else if (currentVoltage < L11_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10);
        } else if (currentVoltage < L12_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11);
        } else if (currentVoltage < L13_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12);
        } else if (currentVoltage < L14_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13);
        } else if (currentVoltage < L15_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14);
        } else if (currentVoltage < L16_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);
        } else if (currentVoltage < L17_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
        } else if (currentVoltage < L18_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17);
        } else {
            enableAllSprites();
        }
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
