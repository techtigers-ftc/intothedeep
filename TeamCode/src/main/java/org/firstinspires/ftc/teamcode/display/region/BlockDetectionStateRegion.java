package org.firstinspires.ftc.teamcode.display.region;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.comp.Check;

import org.firstinspires.ftc.teamcode.display.sprite.CheckmarkSprite;
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
    private final CheckmarkSprite detectedSprite;
    private final UpArrowSprite upArrowSprite;
    private final Sprite[] sprites;
    private ElapsedTime timer;

    /**
     * Creates a new BlockDetectionStateRegion
     *
     * @param x          the x position of the region on the display
     * @param y          the y position of the region on the display
     * @param robotState the robot state
     */
    public BlockDetectionStateRegion(int x, int y, RobotState robotState) {
        super(x, y, 5, 7);
        this.robotState = robotState;

        notDetectedSprite = new XSprite(0, 3, 4, 4);
//        detectedSprite = new RectangleSprite(1, 0, 2, 4);
        detectedSprite = new CheckmarkSprite(0, 2, 4, 3);
        upArrowSprite = new UpArrowSprite(1, 0, 3, 5);

        notDetectedSprite.setColor(Color.ORANGE);
        detectedSprite.setColor(Color.WHITE);
        upArrowSprite.setColor(Color.ORANGE);

        this.sprites = new Sprite[]{notDetectedSprite, detectedSprite, upArrowSprite};
        timer = new ElapsedTime();
        timer.reset();
    }


    @Override
    public void update() {
        BlockDetectionState blockDetectionState = robotState.getCoarseBlockDetectionState();

        if (blockDetectionState == BlockDetectionState.DETECTED) {
            disableAllButSelectedSprites(1);
        } else if (blockDetectionState == BlockDetectionState.TOO_FAR) {
            disableAllButSelectedSprites(2);
        } else {
            disableAllButSelectedSprites(0);
        }
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
