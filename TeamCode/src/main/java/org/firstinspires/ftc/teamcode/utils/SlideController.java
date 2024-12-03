package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * This class encapsulates the logic for setting slides to a given position into one class so that
 * it can be used by other subsystems (namely the intake and dropper subsystems) without
 * duplicating code.
 */
public class SlideController {
    public double targetTicks;
    private double ticksPerInch;
    private PIDFController forwardPIDFController;
    private final double forwardKf;

    /**
     * Initializes the SlideController and sets two different PIDs for forward and reverse movement of the slides
     *
     * @param ticksPerInch the number of encoder ticks per inch of slide travel
     * @param forwardPIDF  the initial PIDF coefficients for forward movement of the slide
     */
    public SlideController(double ticksPerInch, PIDFCoefficients forwardPIDF) {
        this.ticksPerInch = ticksPerInch;
        this.forwardPIDFController = new PIDFController(forwardPIDF.p, forwardPIDF.i, forwardPIDF.d, 0);
        forwardKf = forwardPIDF.f;

        targetTicks = 0;
    }

    /**
     * Allows users to change/set the PID coefficients for the forward controller
     *
     * @param coefficients the PID coefficients to set the forward controller to
     */
    public void setForwardPIDFCoefficients(PIDFCoefficients coefficients) {
        forwardPIDFController.setPIDF(coefficients.p, coefficients.i, coefficients.d, coefficients.f);
    }

    /**
     * Sets the tolerance for both PID controllers
     *
     * @param tolerance the tolerance (position) for the controllers
     */
    public void setTolerance(double tolerance) {
        forwardPIDFController.setTolerance(tolerance);
    }


    /**
     * Moves the slides to a specific, absolute position
     *
     * @param targetDistance the target distance in inches to move the slides to
     */
    public void moveToInches(double targetDistance) {
        targetTicks = targetDistance * ticksPerInch;

    }

    /**
     * Calculates and returns the needed motor power to move slides to a given position
     *
     * @param currentTicks the current encoder ticks of the slides
     * @return the motor power needed to move the slides to the target position
     */
    public double calculateMotorPowers(double currentTicks) {
        double currentPower = forwardPIDFController.calculate(currentTicks, targetTicks);
        RobotLog.dd("tt-ss", "PID Power: [%s]", String.valueOf(currentPower + forwardKf));
        return currentPower + forwardKf;
    }
}