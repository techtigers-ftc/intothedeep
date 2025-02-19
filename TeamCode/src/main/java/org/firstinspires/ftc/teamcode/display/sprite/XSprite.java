package org.firstinspires.ftc.teamcode.display.sprite;


import team.techtigers.core.display.Color;
import team.techtigers.core.display.Sprite;

/**
 * A class which represents a x shaped sprite
 */
public class XSprite extends Sprite {

    /**
     * Creates a new x sprite
     *
     * @param x     the x coordinate of the bottom left corner of the sprite within the region
     * @param y     the y coordinate of the bottom left corner of the sprite within the region
     * @param width the width of the x
     * @param height the height of the x
     */
    public XSprite(int x, int y, int width, int height) {
        super(x, y, width, height);
    }


    @Override
    protected void showSprite(Color[][] leds) {
        for (int i = 0; i < getWidth(); i++) {
            leds[getX() + i][getY() + i] = getColor();
            leds[getX() + i][getY() + getWidth() - 1 - i] = getColor();
        }
    }
}
