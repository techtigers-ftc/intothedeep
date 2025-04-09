package org.firstinspires.ftc.teamcode.display.sprite;


import team.techtigers.core.display.Color;
import team.techtigers.core.display.Sprite;

/**
 * A class which represents a down arrow shaped sprite
 */
public class DownArrowSprite extends Sprite {

    /**
     * Creates a new down arrow sprite
     *
     * @param x     the x coordinate of the bottom left corner of the sprite within the region
     * @param y     the y coordinate of the bottom left corner of the sprite within the region
     * @param width the width of the arrow
     * @param height the height of the arrow
     */
    public DownArrowSprite(int x, int y, int width, int height) {
        super(x, y, width, height);
    }


    @Override
    protected void showSprite(Color[][] leds) {
        for (int i=0; i<getHeight(); i++) {
            leds[getX() + 2][getY() + i] = getColor();
        }
        leds[getX()][getY()+2] = getColor();
        leds[getX()+1][getY()+1] = getColor();
        leds[getX()+3][getY()+1] = getColor();
        leds[getX()+4][getY()+2] = getColor();
    }
}
