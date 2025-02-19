package org.firstinspires.ftc.teamcode.display.sprite.numbers;


import team.techtigers.core.display.Color;
import team.techtigers.core.display.Sprite;

/**
 * A class which represents a sprite in the shape of a 3
 */
public class ThreeSprite extends Sprite {

    /**
     * Creates a new 3 sprite
     *
     * @param x     the x coordinate of the bottom left corner of the sprite within the region
     * @param y     the y coordinate of the bottom left corner of the sprite within the region
     */
    public ThreeSprite(int x, int y) {
        super(x, y, 3, 5);
    }

    @Override
    protected void showSprite(Color[][] leds) {
        for (int i = 0; i < getHeight(); i += 2) {
            for (int j = 0; j < getWidth(); j++) {
                leds[getX() + j][getY() + i] = getColor();
            }
        }
        leds[getX() + 2][getY() + 1] = getColor();
        leds[getX() + 2][getY() + 3] = getColor();
    }
}
