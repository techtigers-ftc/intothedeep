package org.firstinspires.ftc.teamcode.display.region;

import org.firstinspires.ftc.teamcode.display.sprite.numbers.EightSprite;
import org.firstinspires.ftc.teamcode.display.sprite.numbers.FiveSprite;
import org.firstinspires.ftc.teamcode.display.sprite.numbers.FourSprite;
import org.firstinspires.ftc.teamcode.display.sprite.numbers.NineSprite;
import org.firstinspires.ftc.teamcode.display.sprite.numbers.OneSprite;
import org.firstinspires.ftc.teamcode.display.sprite.numbers.SevenSprite;
import org.firstinspires.ftc.teamcode.display.sprite.numbers.SixSprite;
import org.firstinspires.ftc.teamcode.display.sprite.numbers.ThreeSprite;
import org.firstinspires.ftc.teamcode.display.sprite.numbers.TwoSprite;
import org.firstinspires.ftc.teamcode.display.sprite.numbers.ZeroSprite;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.Sprite;

public class CountdownTimerRegion extends DisplayRegion {
    private final ZeroSprite tensZeroSprite;
    private final ZeroSprite onesZeroSprite;
    private final OneSprite tensOneSprite;
    private final OneSprite onesOneSprite;
    private final TwoSprite tensTwoSprite;
    private final TwoSprite onesTwoSprite;
    private final ThreeSprite tensThreeSprite;
    private final ThreeSprite onesThreeSprite;
    private final FourSprite tensFourSprite;
    private final FourSprite onesFourSprite;
    private final FiveSprite tensFiveSprite;
    private final FiveSprite onesFiveSprite;
    private final SixSprite tensSixSprite;
    private final SixSprite onesSixSprite;
    private final SevenSprite tensSevenSprite;
    private final SevenSprite onesSevenSprite;
    private final EightSprite tensEightSprite;
    private final EightSprite onesEightSprite;
    private final NineSprite tensNineSprite;
    private final NineSprite onesNineSprite;
    private final RobotState robotState;
    private final Sprite[] sprites;

    /**
     * Creates a new countdown timer region
     *
     * @param robotState the robot state used to access the intake height
     */
    public CountdownTimerRegion(int x, int y, RobotState robotState) {
        super(x, y, 7, 5);
        this.robotState = robotState;
        tensZeroSprite = new ZeroSprite(0, 0);
        onesZeroSprite = new ZeroSprite(4, 0);
        tensOneSprite = new OneSprite(0, 0);
        onesOneSprite = new OneSprite(4, 0);
        tensTwoSprite = new TwoSprite(0, 0);
        onesTwoSprite = new TwoSprite(4, 0);
        tensThreeSprite = new ThreeSprite(0, 0);
        onesThreeSprite = new ThreeSprite(4, 0);
        tensFourSprite = new FourSprite(0, 0);
        onesFourSprite = new FourSprite(4, 0);
        tensFiveSprite = new FiveSprite(0, 0);
        onesFiveSprite = new FiveSprite(4, 0);
        tensSixSprite = new SixSprite(0, 0);
        onesSixSprite = new SixSprite(4, 0);
        tensSevenSprite = new SevenSprite(0, 0);
        onesSevenSprite = new SevenSprite(4, 0);
        tensEightSprite = new EightSprite(0, 0);
        onesEightSprite = new EightSprite(4, 0);
        tensNineSprite = new NineSprite(0, 0);
        onesNineSprite = new NineSprite(4, 0);
        sprites = new Sprite[]{tensZeroSprite, tensOneSprite, tensTwoSprite, tensThreeSprite, tensFourSprite,
                tensFiveSprite, tensSixSprite, tensSevenSprite, tensEightSprite, tensNineSprite,
                onesZeroSprite, onesOneSprite, onesTwoSprite, onesThreeSprite, onesFourSprite,
                onesFiveSprite, onesSixSprite, onesSevenSprite, onesEightSprite, onesNineSprite};
    }

    @Override
    public void update() {
        int timerTime = (int) robotState.getRunTime() / 1000;
        if (robotState.isAuto()) {
            timerTime += 90;
        }

        int displayedTime;
        Color color;
        if (timerTime > 90 && timerTime < 120) {
            displayedTime = 120 - timerTime;
            color = Color.ORANGE;
        } else if (timerTime < 90) {
            displayedTime = 90 - timerTime;
            color = Color.PURPLE;
        } else {
            displayedTime = 77;
            color = Color.BLACK;
        }

        int onesDigit = displayedTime % 10;
        int tensDigit = Math.floorDiv(displayedTime, 10);
        for (Sprite sprite : sprites) {
            sprite.disable();
            sprite.setColor(color);
        }
        sprites[tensDigit].enable();
        sprites[onesDigit + 10].enable();
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
