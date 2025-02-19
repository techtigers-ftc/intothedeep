package org.firstinspires.ftc.teamcode.display.sprite;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.Sprite;

/**
 * A class which represents a diagonal rectangle shaped sprite
 */
public class DiagonalRectangleSprite extends Sprite {

    /**
     * Creates a new diagonal rectangle sprite
     *
     * @param x      the x coordinate of the bottom left corner of the sprite within the region
     * @param y      the y coordinate of the bottom left corner of the sprite within the region
     */
    public DiagonalRectangleSprite(int x, int y) {
        super(x, y, 2, 2);
    }

    @Override
    protected void showSprite(Color[][] leds) {
        leds[getX()][getY()] = getColor();
        leds[getX() + 1][getY() + 1] = getColor();
    }
}