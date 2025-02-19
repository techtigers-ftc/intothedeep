package org.firstinspires.ftc.teamcode.display.sprite;


import team.techtigers.core.display.Color;
import team.techtigers.core.display.Sprite;

/**
 * A class which represents a up arrow shaped sprite
 */
public class UpArrowSprite extends Sprite {

    /**
     * Creates a new up arrow sprite
     *
     * @param x     the x coordinate of the bottom left corner of the sprite within the region
     * @param y     the y coordinate of the bottom left corner of the sprite within the region
     * @param width the width of the arrow
     * @param height the height of the arrow
     */
    public UpArrowSprite(int x, int y, int width, int height) {
        super(x, y, width, height);
    }


    @Override
    protected void showSprite(Color[][] leds) {
        for (int i = 0; i < getHeight(); i++) {
            leds[getX() + 1][getY() + i] = getColor();
        }
        for (int i = 0; i < getWidth(); i++) {
            leds[getX() + i][getY() + getHeight() - 1] = getColor();
        }
        leds[getX() + 1][getY() + getHeight()] = getColor();
    }
}
