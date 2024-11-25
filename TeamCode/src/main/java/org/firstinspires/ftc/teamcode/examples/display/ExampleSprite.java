package org.firstinspires.ftc.teamcode.examples.display;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.Sprite;

/**
 * A class which represents a sprite shaped like the number 1
 */
public class ExampleSprite extends Sprite {

    /**
     * Creates a new example sprite
     *
     * @param x the x coordinate of the bottom left corner of the sprite within the region
     * @param y the y coordinate of the bottom left corner of the sprite within the region
     */
    public ExampleSprite(int x, int y) {
        super(x, y, 2, 5);
    }


    @Override
    protected void showSprite(Color[][] leds) {
        leds[getX() + 1][getY()] = getColor();
        leds[getX() + 1][getY() + 1] = getColor();
        leds[getX() + 1][getY() + 2] = getColor();
        leds[getX() + 1][getY() + 3] = getColor();
        leds[getX() + 1][getY() + 4] = getColor();
        leds[getX()][getY() + 3] = getColor();
    }
}