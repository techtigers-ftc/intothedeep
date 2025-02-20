package org.firstinspires.ftc.teamcode.display.region;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.display.sprite.DiagonalBlockSpriteLeft;
import org.firstinspires.ftc.teamcode.display.sprite.DiagonalBlockSpriteRight;
import org.firstinspires.ftc.teamcode.display.sprite.HollowRectangleWithCrosshairSprite;
import org.firstinspires.ftc.teamcode.display.sprite.RectangleSprite;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.Sprite;

public class VisionStreamRegion extends DisplayRegion {
    private final HollowRectangleWithCrosshairSprite frame;
    private final RectangleSprite verticalBlock;
    private final RectangleSprite horizontalBlock;
    private final DiagonalBlockSpriteLeft diagonalBlockLeft;
    private final DiagonalBlockSpriteRight diagonalBlockRight;
    private final Sprite[] sprites;
    private RobotState robotState;
    private int blockX;
    private int blockY;
    private double blockOrientation;
    private double LATERAL_INCHES_LIMIT = 6;
    private double VERTICAL_INCHES_LIMIT = 2;

    public VisionStreamRegion(int x, int y, RobotState robotState) {
        super(x, y, 13, 8);
        this.robotState = robotState;
        frame = new HollowRectangleWithCrosshairSprite(0, 0, 13, 8);

        verticalBlock = new RectangleSprite(1, 1, 2, 4);
        horizontalBlock = new RectangleSprite(1, 1, 4, 2);
        diagonalBlockLeft = new DiagonalBlockSpriteLeft(1, 1);
        diagonalBlockRight = new DiagonalBlockSpriteRight(1, 1);

        frame.setColor(Color.WHITE);
        verticalBlock.setColor(Color.YELLOW);
        horizontalBlock.setColor(Color.YELLOW);
        diagonalBlockRight.setColor(Color.YELLOW);
        diagonalBlockLeft.setColor(Color.YELLOW);
        frame.enable();

        sprites = new Sprite[]{verticalBlock, horizontalBlock, diagonalBlockLeft, diagonalBlockRight, frame};
    }

    @Override
    public void update() {
        blockY = (int) ((robotState.getBlockForwardFine() + (VERTICAL_INCHES_LIMIT / 2)) / (VERTICAL_INCHES_LIMIT) * 2);
        blockX = (int) ((robotState.getBlockLateralFine() + (LATERAL_INCHES_LIMIT / 2)) / (LATERAL_INCHES_LIMIT) * 10);
        blockOrientation = robotState.getBlockOrientation();

        if (0 <= blockOrientation && blockOrientation < 22.5) {
            disableAllBlocks();
            horizontalBlock.setPosition(Range.clip(blockX, 1, 8), Range.clip(blockY + 2, 1, 5));
            horizontalBlock.enable();
        } else if (22.5 < blockOrientation && blockOrientation < 67.5) {
            disableAllBlocks();
            diagonalBlockLeft.setPosition(Range.clip(blockX, 1, 9), Range.clip(blockY + 1, 1, 3));
            diagonalBlockLeft.enable();
        } else if (67.5 < blockOrientation && blockOrientation < 112.5) {
            disableAllBlocks();
            verticalBlock.setPosition(Range.clip(blockX + 1, 1, 10), Range.clip(blockY + 1, 1, 3));
            verticalBlock.enable();
        } else if (112.5 < blockOrientation && blockOrientation < 157.5) {
            disableAllBlocks();
            diagonalBlockRight.setPosition(Range.clip(blockX, 1, 9), Range.clip(blockY + 1, 1, 3));
            diagonalBlockRight.enable();
        } else if (157.5 < blockOrientation && blockOrientation <= 180) {
            disableAllBlocks();
            horizontalBlock.setPosition(Range.clip(blockX, 1, 8), Range.clip(blockY + 2, 1, 5));
            horizontalBlock.enable();
        }
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }

    private void disableAllBlocks() {
        verticalBlock.disable();
        horizontalBlock.disable();
        diagonalBlockLeft.disable();
        diagonalBlockRight.disable();
    }
}
