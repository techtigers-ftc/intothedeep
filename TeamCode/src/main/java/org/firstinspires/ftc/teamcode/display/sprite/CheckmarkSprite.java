package org.firstinspires.ftc.teamcode.display.sprite;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.Sprite;

/**
 * A class which represents a checkmark shaped sprite
 */
public class CheckmarkSprite extends Sprite {
    /**
     * Creates a new checkmark sprite
     * @param x     the x coordinate of the bottom left corner of the sprite within the region
     * @param y     the y coordinate of the bottom left corner of the sprite within the region
     * @param width the width of the line
     * @param height the height of the line
     */
    public CheckmarkSprite(int x, int y, int width, int height) {
        super(x, y, width, height);
    }


    @Override
    protected void showSprite(Color[][] leds) {
        leds[getX()][getY() + 1] = getColor();
        leds[getX() + 1][getY()] = getColor();
        leds[getX() + 2][getY() + 1] = getColor();
        leds[getX() + 3][getY() + 2] = getColor();
    }
}
