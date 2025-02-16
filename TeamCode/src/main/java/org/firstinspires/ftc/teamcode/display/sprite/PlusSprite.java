package org.firstinspires.ftc.teamcode.display.sprite;


import team.techtigers.core.display.Color;
import team.techtigers.core.display.Sprite;

/**
 * A class which represents a plus shaped sprite
 */
public class PlusSprite extends Sprite {

    /**
     * Creates a new plus sprite
     *
     * @param x     the x coordinate of the bottom left corner of the sprite within the region
     * @param y     the y coordinate of the bottom left corner of the sprite within the region
     */
    public PlusSprite(int x, int y) {
        super(x, y, 3,3);
    }


    @Override
    protected void showSprite(Color[][] leds) {
        for (int i = 0; i < 3; i++) {
            leds[getX() + i][getY() + 1] = getColor();
            leds[getX() + 1][getY() + i] = getColor();
        }
    }
}
