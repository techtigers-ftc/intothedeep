package org.firstinspires.ftc.teamcode.display.region;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.display.sprite.RectangleSprite;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.statemachine.State;
import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.Sprite;

/**
 * A region that flashes the sprite when the intake picks up a block
 */
public class AutoStateRegion extends DisplayRegion {
    private final RobotState robotState;
    private final RectangleSprite stateSprite;
    private final RectangleSprite debugSprite;
    private final Sprite[] sprites;
    private State<AutoState> currentState;
    private ElapsedTime timer;

    /**
     * Creates a new IntakeFlashbangRegion
     *
     * @param x          the x position of the region on the display
     * @param y          the y position of the region on the display
     * @param robotState the robot state
     */
    public AutoStateRegion(int x, int y, RobotState robotState) {
        super(x, y, 2, 8);
        this.robotState = robotState;

        stateSprite = new RectangleSprite(0, 0, 2, 4);
        stateSprite.enable();
        debugSprite = new RectangleSprite(0, 4, 2, 4);
        debugSprite.enable();

        this.sprites = new Sprite[]{stateSprite};
        this.currentState = robotState.getCurrentAutoState();
        this.timer = new ElapsedTime();
        stateSprite.setColor(Color.GREEN);
        stateSprite.setColor(Color.BLACK);
        timer.reset();
    }


    @Override
    public void update() {
        if (this.currentState != robotState.getCurrentAutoState()) {
            stateSprite.setColor(Color.ORANGE);
            timer.reset();
            timer.startTime();
        }else if (timer.milliseconds() > 500 && this.currentState == robotState.getCurrentAutoState()) {
            stateSprite.setColor(Color.GREEN);
        }
        debugSprite.setColor(robotState.getDebugColor());
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
