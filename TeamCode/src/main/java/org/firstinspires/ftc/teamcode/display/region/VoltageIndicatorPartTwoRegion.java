package org.firstinspires.ftc.teamcode.display.region;

import org.firstinspires.ftc.teamcode.display.sprite.RectangleSprite;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.Sprite;

/**
 * A region which shows an indicator that shows the current status of the battery's voltage
 */
public class VoltageIndicatorPartTwoRegion extends DisplayRegion {
    private final RobotState robotState;
    private final RectangleSprite L7Sprite;
    private final RectangleSprite L8Sprite;
    private final RectangleSprite L9Sprite;
    private final RectangleSprite L10Sprite;
    private final RectangleSprite L11Sprite;
    private final RectangleSprite L12Sprite;

    private final static double MIN_VOLTAGE_THRESHOLD = 8;
    private final static double MAX_VOLTAGE_THRESHOLD = 11.66;

    private final static double STEP = (MAX_VOLTAGE_THRESHOLD - MIN_VOLTAGE_THRESHOLD) / 18;

    private final static double L7_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 7 * STEP;
    private final static double L8_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 8 * STEP;
    private final static double L9_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 9 * STEP;
    private final static double L10_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 10 * STEP;
    private final static double L11_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 11 * STEP;
    private final static double L12_THRESHOLD = MIN_VOLTAGE_THRESHOLD + 12 * STEP;

    private final Sprite[] sprites;

    /**
     * Creates a new VoltageIndicatorRegionPartTwo
     *
     * @param x          the x position of the region
     * @param y          the y position of the region
     * @param robotState the robot state
     */
    public VoltageIndicatorPartTwoRegion(int x, int y, RobotState robotState) {
        super(x, y, 8, 2);
        this.robotState = robotState;
        L7Sprite = new RectangleSprite(0, 0, 2, 1);
        L8Sprite = new RectangleSprite(2, 0, 1, 2);
        L9Sprite = new RectangleSprite(3, 0, 1, 2);
        L10Sprite = new RectangleSprite(4, 0, 1, 2);
        L11Sprite = new RectangleSprite(5, 0, 1, 2);
        L12Sprite = new RectangleSprite(6, 0, 2, 1);


        sprites = new Sprite[]{L7Sprite, L8Sprite, L9Sprite, L10Sprite, L11Sprite, L12Sprite};

        for (int i = 0; i < 5; i++) {
            sprites[i].setColor(Color.YELLOW);
        }
        for (int i = 0; i < 1; i++) {
            sprites[i + 5].setColor(Color.GREEN);
        }
    }


    @Override
    public void update() {
        double currentVoltage = robotState.getVoltage();
        if (currentVoltage < L7_THRESHOLD) {
            disableAllButSelectedSprites(0);
        } else if (currentVoltage < L8_THRESHOLD) {
            disableAllButSelectedSprites(0, 1);
        } else if (currentVoltage < L9_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2);
        } else if (currentVoltage < L10_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3);
        } else if (currentVoltage < L11_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4);
        } else if (currentVoltage < L12_THRESHOLD) {
            disableAllButSelectedSprites(0, 1, 2, 3, 4, 5);
        }  else {
            enableAllSprites();
        }
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
