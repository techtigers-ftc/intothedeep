package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * This class encapsulates the logic for setting slides to a given position into one class so that
 * it can be used by other subsystems (namely the intake and dropper subsystems) without
 * duplicating code.
 */
public class SlideController {
    private double targetTicks;
    private double ticksPerInch;
    private PIDFController forwardPIDFController;
    private PIDFController reversePIDFController;

    /**
     * Initializes the SlideController
     *
     * @param ticksPerInch the number of encoder ticks per inch of slide travel
     * @param forwardPIDF the initial PIDF coefficients for forward movement of the slide
     * @param reversePIDF the initial PIDF coefficients for reverse movement of the slide
     */
    public SlideController(double ticksPerInch, PIDFCoefficients forwardPIDF, PIDFCoefficients reversePIDF) {
        this.ticksPerInch = ticksPerInch;
        this.forwardPIDFController = new PIDFController(forwardPIDF.p, forwardPIDF.i, forwardPIDF.d, forwardPIDF.f);
        this.reversePIDFController = new PIDFController(reversePIDF.p, reversePIDF.i, reversePIDF.d, reversePIDF.f);
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
     * Allows users to change/set the PID coefficients for the reverse controller
     *
     * @param coefficients the PID coefficients to set the reverse controller to
     */
    public void setReversePIDFCoefficients(PIDFCoefficients coefficients) {
        reversePIDFController.setPIDF(coefficients.p, coefficients.i, coefficients.d, coefficients.f);
    }

    /**
     * Moves the slides to a specific, absolute position
     *
     * @param targetDistance the target distance in inches to move the slides to
     */
    public void moveTo(double targetDistance) {
        this.targetTicks = targetDistance * ticksPerInch;
    }

    /**
     * Moves the slides to a relative position based on their current position
     *
     * @param relativeDistance the relative distance in inches to move the slides
     */
    public void moveToRelative(double relativeDistance) {
        this.targetTicks += relativeDistance * ticksPerInch;
    }

    /**
     * Calculates and returns the needed motor power to move slides to a given position
     *
     * @param currentTicks the current encoder ticks of the slides
     * @return the motor power needed to move the slides to the target position
     */
    public double calculateMotorPowers(double currentTicks) {
        if (currentTicks > targetTicks) {
            return reversePIDFController.calculate(currentTicks, targetTicks);
        } else {
            return forwardPIDFController.calculate(currentTicks, targetTicks);
        }
    }
}