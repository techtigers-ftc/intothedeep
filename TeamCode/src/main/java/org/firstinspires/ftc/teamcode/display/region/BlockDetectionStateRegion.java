package org.firstinspires.ftc.teamcode.display.region;

import org.firstinspires.ftc.teamcode.display.sprite.RectangleSprite;
import org.firstinspires.ftc.teamcode.display.sprite.UpArrowSprite;
import org.firstinspires.ftc.teamcode.display.sprite.XSprite;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.Sprite;

/**
 * A region that displays the current drive gear of the robot
 */
public class BlockDetectionStateRegion extends DisplayRegion {
    private final RobotState robotState;
    private final XSprite notDetectedSprite;
    private final RectangleSprite detectedSprite;
    private final UpArrowSprite upArrowSprite;
    private final Sprite[] sprites;

    /**
     * Creates a new BlockDetectionStateRegion
     *
     * @param x          the x position of the region on the display
     * @param y          the y position of the region on the display
     * @param robotState the robot state
     */
    public BlockDetectionStateRegion(int x, int y, RobotState robotState) {
        super(x, y, 3, 4);
        this.robotState = robotState;

        notDetectedSprite = new XSprite(0, 0, 3, 3);
        detectedSprite = new RectangleSprite(0, 0, 3, 4);
        upArrowSprite = new UpArrowSprite(0, 0, 3, 4);

        notDetectedSprite.setColor(Color.ORANGE);
        detectedSprite.setColor(Color.GREEN);
        upArrowSprite.setColor(Color.ORANGE);

        this.sprites = new Sprite[]{notDetectedSprite, detectedSprite, upArrowSprite};
    }


    @Override
    public void update() {
        BlockDetectionState blockDetectionState = robotState.getCoarseBlockDetectionState();
        if (blockDetectionState == BlockDetectionState.DETECTED) {
            disableAllButSelectedSprites(1);
        } else if (blockDetectionState == BlockDetectionState.NOT_DETECTED) {
            disableAllButSelectedSprites(1);
        }
        // TODO: Fix once main gets the new block detection state
//        else if(blockDetectionState == BlockDetectionState.TOO_FAR) {
//            enableAllButSelectedSprite(2);
//        }
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
