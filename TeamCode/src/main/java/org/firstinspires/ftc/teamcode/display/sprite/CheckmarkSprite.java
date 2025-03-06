package org.firstinspires.ftc.teamcode.display.sprite;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.Sprite;

/**
 * A class which represents a checkmark shaped sprite
 */
public class CheckmarkSprite extends Sprite {
    /**
     * Creates a new checkmark sprite
     *
     * @param x the x coordinate of the bottom left corner of the sprite within the region
     * @param y the y coordinate of the bottom left corner of the sprite within the region
     */
    public CheckmarkSprite(int x, int y) {
        super(x, y, 9, 6);
    }


    @Override
    protected void showSprite(Color[][] leds) {
        leds[getX()][getY() + 2] = getColor();
        leds[getX() + 1][getY() + 1] = getColor();
        leds[getX() + 2][getY()] = getColor();
        leds[getX() + 3][getY() + 1] = getColor();
        leds[getX() + 4][getY() + 2] = getColor();
        leds[getX() + 5][getY() + 3] = getColor();
        leds[getX() + 6][getY() + 4] = getColor();
        leds[getX() + 7][getY() + 5] = getColor();
    }
}
