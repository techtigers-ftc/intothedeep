package org.firstinspires.ftc.teamcode.display.region;

import org.firstinspires.ftc.teamcode.display.sprite.VoltageMeterSprite;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.Sprite;

/**
 * A region which shows an indicator that shows the current status of the battery's voltage
 */
public class VoltageIndicatorRegion extends DisplayRegion {
    private final static double MIN_VOLTAGE_THRESHOLD = 9.5;
    private final static double MAX_VOLTAGE_THRESHOLD = 13.5;
    private final static double STEP = (MAX_VOLTAGE_THRESHOLD - MIN_VOLTAGE_THRESHOLD) / 10;
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
    private final RobotState robotState;
    private final VoltageMeterSprite L1Sprite;
    private final VoltageMeterSprite L2Sprite;
    private final VoltageMeterSprite L3Sprite;
    private final VoltageMeterSprite L4Sprite;
    private final VoltageMeterSprite L5Sprite;
    private final VoltageMeterSprite L6Sprite;
    private final VoltageMeterSprite L7Sprite;
    private final VoltageMeterSprite L8Sprite;
    private final VoltageMeterSprite L9Sprite;
    private final VoltageMeterSprite L10Sprite;
    private final Sprite[] sprites;

    /**
     * Creates a new VoltageIndicatorRegion
     *
     * @param x          the x position of the region
     * @param y          the y position of the region
     * @param robotState the robot state
     */
    public VoltageIndicatorRegion(int x, int y, RobotState robotState) {
        super(x, y, 11, 2);
        this.robotState = robotState;
        L1Sprite = new VoltageMeterSprite(0, 0);
        L2Sprite = new VoltageMeterSprite(1, 0);
        L3Sprite = new VoltageMeterSprite(2, 0);
        L4Sprite = new VoltageMeterSprite(3, 0);
        L5Sprite = new VoltageMeterSprite(4, 0);
        L6Sprite = new VoltageMeterSprite(5, 0);
        L7Sprite = new VoltageMeterSprite(6, 0);
        L8Sprite = new VoltageMeterSprite(7, 0);
        L9Sprite = new VoltageMeterSprite(8, 0);
        L10Sprite = new VoltageMeterSprite(9, 0);


        sprites = new Sprite[]{L1Sprite, L2Sprite, L3Sprite, L4Sprite,
                L5Sprite, L6Sprite, L7Sprite, L8Sprite, L9Sprite, L10Sprite
        };

        for (int i = 0; i < 3; i++) {
            sprites[i].setColor(Color.PINK);
        }
        for (int i = 0; i < 3; i++) {
            sprites[i + 3].setColor(Color.YELLOW);
        }
        for (int i = 0; i < 4; i++) {
            sprites[i + 6].setColor(Color.GREEN);
        }
    }


    @Override
    public void update() {
        double currentVoltage = robotState.getVoltage();

        if (currentVoltage < L1_THRESHOLD) {
            disableAllButSelectedSprites(0);
        } else if (currentVoltage < L2_THRESHOLD) {
            disableAllButSelectedSprites(0);
        } else if (currentVoltage < L3_THRESHOLD) {
            disableAllButSelectedSprites(0, 1);
        } else if (currentVoltage < L4_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2);
        } else if (currentVoltage < L5_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3);
        } else if (currentVoltage < L6_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4);
        } else if (currentVoltage < L7_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4, 5);
        } else if (currentVoltage < L8_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4, 5, 6);
        } else if (currentVoltage < L9_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4, 5, 6, 7);
        } else if (currentVoltage < L10_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4, 5, 6, 7, 8);
        } else {
            enableAllSprites();
        }
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
