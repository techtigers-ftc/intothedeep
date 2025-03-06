package org.firstinspires.ftc.teamcode.display.region;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.display.sprite.CheckmarkSprite;
import org.firstinspires.ftc.teamcode.display.sprite.DiagonalBlockSpriteLeft;
import org.firstinspires.ftc.teamcode.display.sprite.DiagonalBlockSpriteRight;
import org.firstinspires.ftc.teamcode.display.sprite.FrameSprite;
import org.firstinspires.ftc.teamcode.display.sprite.RectangleSprite;
import org.firstinspires.ftc.teamcode.display.sprite.XSprite;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.Sprite;

public class VisionStreamRegion extends DisplayRegion {
    private final FrameSprite frame;
    private final RectangleSprite verticalBlock;
    private final RectangleSprite horizontalBlock;
    private final DiagonalBlockSpriteLeft diagonalBlockLeft;
    private final DiagonalBlockSpriteRight diagonalBlockRight;
    private final CheckmarkSprite checkmark;
    private final XSprite noBlockDetected;

    private final Sprite[] sprites;
    private RobotState robotState;

    private int blockX;
    private int blockY;
    private double blockOrientation;
    private double LATERAL_INCHES_LIMIT = 4.6;
    private double VERTICAL_INCHES_LIMIT = 2;

    public VisionStreamRegion(int x, int y, RobotState robotState) {
        super(x, y, 13, 8);
        this.robotState = robotState;
        frame = new FrameSprite(0, 0, 13, 8);

        verticalBlock = new RectangleSprite(1, 1, 2, 4);
        horizontalBlock = new RectangleSprite(1, 1, 4, 2);
        diagonalBlockLeft = new DiagonalBlockSpriteLeft(1, 1);
        diagonalBlockRight = new DiagonalBlockSpriteRight(1, 1);
        noBlockDetected = new XSprite(3, 1, 6, 6);
        checkmark = new CheckmarkSprite(2, 1);

        frame.setColor(Color.WHITE);
        verticalBlock.setColor(Color.YELLOW);
        horizontalBlock.setColor(Color.YELLOW);
        diagonalBlockRight.setColor(Color.YELLOW);
        diagonalBlockLeft.setColor(Color.YELLOW);
        noBlockDetected.setColor(Color.ORANGE);
        checkmark.setColor(Color.GREEN);
        frame.enable();

        sprites = new Sprite[]{verticalBlock, horizontalBlock, diagonalBlockLeft, diagonalBlockRight, frame, noBlockDetected, checkmark};
    }

    @Override
    public void update() {
        blockX = (int) ((robotState.getBlockLateralFine() + (LATERAL_INCHES_LIMIT / 2)) / (LATERAL_INCHES_LIMIT) * 10);
        blockY = (int) ((robotState.getBlockForwardFine() + (VERTICAL_INCHES_LIMIT / 2)) / (VERTICAL_INCHES_LIMIT) * 3) - 1;
        blockOrientation = robotState.getBlockOrientation();

        if (robotState.getBlockPosition() == RobotBlockPosition.INTAKE) {
            disableAllBlocks();
            checkmark.enable();
        } else if (robotState.getFineBlockDetectionState() == BlockDetectionState.NOT_DETECTED) {
            disableAllBlocks();
            noBlockDetected.enable();
        } else if (0 <= blockOrientation && blockOrientation < 22.5) {
            disableAllBlocks();
            horizontalBlock.setPosition(Range.clip(blockX, 1, 8), Range.clip(blockY + 2, 1, 5));
            horizontalBlock.enable();
            setColor();
        } else if (22.5 < blockOrientation && blockOrientation < 67.5) {
            disableAllBlocks();
            diagonalBlockLeft.setPosition(Range.clip(blockX, 1, 9), Range.clip(blockY + 1, 1, 3));
            diagonalBlockLeft.enable();
            setColor();
        } else if (67.5 < blockOrientation && blockOrientation < 112.5) {
            disableAllBlocks();
            verticalBlock.setPosition(Range.clip(blockX + 1, 1, 10), Range.clip(blockY + 1, 1, 3));
            verticalBlock.enable();
            setColor();
        } else if (112.5 < blockOrientation && blockOrientation < 157.5) {
            disableAllBlocks();
            diagonalBlockRight.setPosition(Range.clip(blockX, 1, 9), Range.clip(blockY + 1, 1, 3));
            diagonalBlockRight.enable();
            setColor();
        } else if (157.5 < blockOrientation && blockOrientation <= 180) {
            disableAllBlocks();
            horizontalBlock.setPosition(Range.clip(blockX, 1, 8), Range.clip(blockY + 2, 1, 5));
            horizontalBlock.enable();
            setColor();
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
        noBlockDetected.disable();
        checkmark.disable();
    }

    // TODO: Make this use color from Govind's pipeline
    private void setColor() {
        if (robotState.getBlockColorPreference() == BlockColorPreference.ALLIANCE) {
            if (robotState.isBlue()) {
                verticalBlock.setColor(Color.BLUE);
                horizontalBlock.setColor(Color.BLUE);
                diagonalBlockLeft.setColor(Color.BLUE);
                diagonalBlockRight.setColor(Color.BLUE);
            } else {
                verticalBlock.setColor(Color.RED);
                horizontalBlock.setColor(Color.RED);
                diagonalBlockLeft.setColor(Color.RED);
                diagonalBlockRight.setColor(Color.RED);
            }
        } else {
            verticalBlock.setColor(Color.YELLOW);
            horizontalBlock.setColor(Color.YELLOW);
            diagonalBlockLeft.setColor(Color.YELLOW);
            diagonalBlockRight.setColor(Color.YELLOW);
        }
    }
}
