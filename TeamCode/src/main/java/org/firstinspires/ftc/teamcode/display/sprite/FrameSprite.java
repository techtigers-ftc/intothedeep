package org.firstinspires.ftc.teamcode.display.sprite;


import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.Sprite;

/**
 * A class which represents a hollow rectangle sprite
 */
public class FrameSprite extends Sprite {

    /**
     * Creates a new hollow rectangle sprite
     *
     * @param x      the x coordinate of the bottom left corner of the sprite within the region
     * @param y      the y coordinate of the bottom left corner of the sprite within the region
     * @param width  the width of the line
     * @param height the height of the line
     */
    public FrameSprite(int x, int y, int width, int height) {
        super(x, y, width, height);
    }


    @Override
    protected void showSprite(Color[][] leds) {
        for (int column = 0; column < getWidth(); column++) {
            leds[getX() + column][getY()] = getColor();
            leds[getX() + column][getY() + getHeight() - 1] = getColor();
        }

        for (int row = 0; row < getHeight(); row++) {
            leds[getX()][getY() + row] = getColor();
            leds[getX() + getWidth() - 1][getY() + row] = getColor();
        }


        leds[getX() + (getWidth() / 2)][getY() + getHeight() / 2] = Color.GREEN;
        leds[getX() + (getWidth() / 2)][getY() + getHeight() / 2 - 1] = Color.GREEN;
    }
}
