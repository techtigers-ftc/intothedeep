package org.firstinspires.ftc.teamcode.display.sprite;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.Sprite;

/**
 * A class which represents a diagonal block shaped sprite pointing to the right
 */
public class DiagonalBlockSpriteLeft extends Sprite {

    /**
     * Creates a new diagonal block sprite
     *
     * @param x      the x coordinate of the bottom left corner of the sprite within the region
     * @param y      the y coordinate of the bottom left corner of the sprite within the region
     */
    public DiagonalBlockSpriteLeft(int x, int y) {
        super(x, y, 3, 4);
    }

    @Override
    protected void showSprite(Color[][] leds) {
        leds[getX()][getY() + 3] = getColor();
        leds[getX()][getY() + 2] = getColor();
        leds[getX() + 1][getY() + 2] = getColor();
        leds[getX() + 1][getY() + 1] = getColor();
        leds[getX() + 2][getY() + 1] = getColor();
        leds[getX() + 2][getY()] = getColor();
    }
}