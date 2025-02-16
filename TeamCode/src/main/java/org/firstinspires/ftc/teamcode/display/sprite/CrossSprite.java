package org.firstinspires.ftc.teamcode.display.sprite;


import team.techtigers.core.display.Color;
import team.techtigers.core.display.Sprite;

/**
 * A class which represents a cross shaped sprite
 */
public class CrossSprite extends Sprite {

    /**
     * Creates a new cross sprite
     *
     * @param x     the x coordinate of the bottom left corner of the sprite within the region
     * @param y     the y coordinate of the bottom left corner of the sprite within the region
     * @param width the width of the line
     * @param height the height of the line
     */
    public CrossSprite(int x, int y, int width, int height) {
        super(x, y, width, height);
    }


    @Override
    protected void showSprite(Color[][] leds) {
        for (int i = 0; i < 3; i++) {
            leds[getX() + i][getY() + i] = getColor();
            leds[getX() + i][getY() + 2 - i] = getColor();
        }
    }
}
