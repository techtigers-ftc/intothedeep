package org.firstinspires.ftc.teamcode.display.sprite.numbers;


import team.techtigers.core.display.Color;
import team.techtigers.core.display.Sprite;

/**
 * A class which represents a cross shaped sprite
 */
public class NineSprite extends Sprite {

    /**
     * Creates a new cross sprite
     *
     * @param x     the x coordinate of the bottom left corner of the sprite within the region
     * @param y     the y coordinate of the bottom left corner of the sprite within the region
     */
    public NineSprite(int x, int y) {
        super(x, y, 3, 5);
    }

    @Override
    protected void showSprite(Color[][] leds) {
        for (int i = 0; i < getHeight(); i++) {
            leds[getX() + 2][getY() + i] = getColor();
        }
        for (int i = 0; i < getWidth(); i++) {
            leds[getX() + i][getY() + 2] = getColor();
            leds[getX() + i][getY() + 4] = getColor();
        }
        leds[getX()][getY() + 3] = getColor();
    }
}
