package org.firstinspires.ftc.teamcode.display.region;

import org.firstinspires.ftc.teamcode.display.sprite.HollowRectangleSprite;
import org.firstinspires.ftc.teamcode.display.sprite.RectangleSprite;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.Sprite;

public class VisionStreamRegion extends DisplayRegion {
    private final HollowRectangleSprite frame;
    private final RectangleSprite block;
    private RobotState robotState;
    private int blockX;
    private int blockY;
    private double blockOrientation;
    private double LATERAL_INCHES_LIMIT = 6;
    private double VERTICAL_INCHES_LIMIT = 5.6;
    private final Sprite[] sprites;

    public VisionStreamRegion(int x, int y, RobotState robotState) {
        super(x, y, 13, 8);
        this.robotState = robotState;
        frame = new HollowRectangleSprite(0, 0, 13, 8);
        block = new RectangleSprite(1, 1, 2, 4);
        frame.setColor(Color.WHITE);
        block.setColor(Color.YELLOW);
        frame.enable();
        block.enable();

        sprites = new Sprite[]{frame, block};
    }
    @Override
    public void update() {
        blockY = (int) ((robotState.getBlockForwardFine() + (VERTICAL_INCHES_LIMIT / 2)) / (VERTICAL_INCHES_LIMIT) * 2);
        blockX = (int) ((robotState.getBlockLateralFine() + (LATERAL_INCHES_LIMIT / 2)) / (LATERAL_INCHES_LIMIT) * 10);
        block.setPosition(blockX + 1, blockY + 1);
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
