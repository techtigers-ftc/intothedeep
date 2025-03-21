package org.firstinspires.ftc.teamcode.display.sprite;


import team.techtigers.core.display.Color;
import team.techtigers.core.display.Sprite;

/**
 * A class which represents a rectangle shaped sprite
 */
public class RectangleSprite extends Sprite {

    /**
     * Creates a new rectangle sprite
     *
     * @param x     the x coordinate of the bottom left corner of the sprite within the region
     * @param y     the y coordinate of the bottom left corner of the sprite within the region
     * @param width the width of the line
     */
    public RectangleSprite(int x, int y, int width, int height) {
        super(x, y, width, height);
    }


    @Override
    protected void showSprite(Color[][] leds) {
        for (int row = 0; row < getHeight(); row++) {
            for (int col = 0; col < getWidth(); col++) {
                leds[getX() + col][getY() + row] = getColor();
            }
        }
    }
}
