package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * This class encapsulates the logic for setting slides to a given position into one class so that
 * it can be used by other subsystems (namely the intake and dropper subsystems) without
 * duplicating code.
 */
public class SlideController {
    public double targetTicks;
    private final double ticksPerInch;
    private final PIDFController pidfController;
    private final double kF;

    /**
     * Initializes the SlideController and PIDs movement of the slides
     *
     * @param ticksPerInch the number of encoder ticks per inch of slide travel
     * @param pidf  the initial PIDF coefficients for movement of the slides
     */
    public SlideController(double ticksPerInch, PIDFCoefficients pidf) {
        this.ticksPerInch = ticksPerInch;
        this.pidfController = new PIDFController(pidf.p, pidf.i, pidf.d, 0);
        kF = pidf.f;

        targetTicks = 0;
    }

    /**
     * Allows users to change/set the PIDF coefficients for the feedback controller
     *
     * @param coefficients the new PIDF coefficients
     */
    public void setPIDFCoefficients(PIDFCoefficients coefficients) {
        pidfController.setPIDF(coefficients.p, coefficients.i, coefficients.d, coefficients.f);
    }

    /**
     * Sets the tolerance for the PID controller
     *
     * @param tolerance the tolerance (position) for the controller
     */
    public void setTolerance(double tolerance) {
        pidfController.setTolerance(tolerance);
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
        double currentPower = pidfController.calculate(currentTicks, targetTicks);
        return currentPower + kF;
    }
}