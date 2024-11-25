package org.firstinspires.ftc.teamcode.examples.display;


import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.Sprite;

/**
 * A class that represents a region in which the number 11 is displayed and moves up and down
 */
public class ExampleRegion extends DisplayRegion {
    private final Sprite[] sprites;
    private final ExampleSprite oneInTensDigit;
    private final ExampleSprite oneInOnesDigit;
    private double lastTime;

    /**
     * Creates a new example region
     *
     * @param x the x coordinate of the bottom left corner of the region
     * @param y the y coordinate of the bottom left corner of the region
     */
    public ExampleRegion(int x, int y) {
        super(x, y, 4, 7); // Specifies the region parameters

        // Creates the sprites
        this.oneInTensDigit = new ExampleSprite(0, 0);
        this.oneInOnesDigit = new ExampleSprite(2, 0);

        // Sets the colors of the sprites
        oneInTensDigit.setColor(Color.PINK);
        oneInOnesDigit.setColor(Color.BLUE);

        // Resets time and adds sprites to a list
        lastTime = System.currentTimeMillis();
        this.sprites = new Sprite[]{oneInTensDigit, oneInOnesDigit};
    }

    @Override
    public void update() {
        // Moves the sprites up and down every second
        if (System.currentTimeMillis() - lastTime > 1000) {
            if (oneInTensDigit.getY() == 1 || oneInOnesDigit.getY() == 1) {
                oneInOnesDigit.setPosition(oneInOnesDigit.getX(), oneInOnesDigit.getY() - 1);
                oneInTensDigit.setPosition(oneInTensDigit.getX(), oneInTensDigit.getY() - 1);
            } else {
                oneInTensDigit.setPosition(oneInTensDigit.getX(), oneInTensDigit.getY() + 1);
                oneInOnesDigit.setPosition(oneInOnesDigit.getX(), oneInOnesDigit.getY() + 1);
            }
            lastTime = System.currentTimeMillis();
        }
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
