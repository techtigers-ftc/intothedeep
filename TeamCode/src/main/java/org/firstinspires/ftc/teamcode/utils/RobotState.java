package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.cv.AbsoluteBlockPosition;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColor;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

import team.techtigers.core.display.Color;
import team.techtigers.core.paths.Waypoint;
import team.techtigers.core.utils.GlobalState;

/**
 * Implementation of a global state for the robot
 */
public class RobotState extends GlobalState {
    private final boolean isBlue;
    private final boolean isAuto;
    private Waypoint robotCurrentPose;
    private Waypoint robotVelocity;
    private Waypoint robotFinalPose;
    private BlockColorPreference blockColorPreference;
    private BlockDetectionState coarseBlockDetectionState;
    private BlockDetectionState fineBlockDetectionState;
    private double blockLateralCoarse;
    private double blockOrientation;
    private double detectedFineBlockOrientation;
    private double blockForwardCoarse;
    private BlockColor intakeBlockColor;
    private BlockColor dropperBlockColor;
    private double blockForwardFine;
    private double blockLateralFine;
    private boolean isHorizontalExtended;
    private ClawState intakeClawState;
    private double intakeClawRotation;
    private double intakeClawPitch;
    private RobotBlockPosition blockPosition;
    private boolean isAscending;
    private boolean isVerticalExtended;
    private boolean isManualIntakeSelected;
    private boolean isBreakBeamEnabled;
    private double dropperClawPitch;
    private double dropperClawRotation;
    private double driverCurrent;
    private double intakeCurrent;
    private double dropperCurrent;
    private ClawState dropperClawState;
    private DropperState dropperState;
    private DriveGears driveGears;
    private IntakeState intakeState;
    private int robotError;
    private double visionIntakeHeading;
    private double voltage;
    private boolean isCameraRunning;
    private boolean isCoarseCameraMode;
    private boolean isVisionAligning;
    private boolean isIntakeTracking;
    private String currentAutoState;
    private String previousAutoState;
    private Color debugColor;
    private double autoRemainingTime;
    private AbsoluteBlockPosition absoluteBlockPosition;
    private Waypoint limelightLastRobotCoords;
    private double intakeSlidePosition;


    private boolean runDistanceSensor;
    private boolean headingLockEnabled;
    private double distanceSensorValue;

    /**
     * Initializes a new RobotState
     */
    public RobotState(boolean isBlue, boolean isAuto) {
        robotCurrentPose = new Waypoint(0, 0, 0);
        robotVelocity = new Waypoint(0, 0, 0);
        robotFinalPose = new Waypoint(0, 0, 0);
        blockColorPreference = BlockColorPreference.ANY;
        coarseBlockDetectionState = BlockDetectionState.NOT_DETECTED;
        fineBlockDetectionState = BlockDetectionState.NOT_DETECTED;
        blockLateralCoarse = 0;
        blockOrientation = 0;
        detectedFineBlockOrientation = 0;
        blockForwardCoarse = 0;
        intakeBlockColor = BlockColor.NONE;
        dropperBlockColor = BlockColor.NONE;
        blockForwardFine = 0;
        blockLateralFine = 0;
        isHorizontalExtended = false;
        intakeClawState = ClawState.OPEN;
        intakeClawRotation = 0;
        intakeClawPitch = 0;
        blockPosition = RobotBlockPosition.NONE;
        isAscending = false;
        isVerticalExtended = false;
        isManualIntakeSelected = false;
        isBreakBeamEnabled = true;
        dropperClawPitch = 0;
        dropperClawRotation = 0;
        dropperClawState = ClawState.OPEN;
        dropperState = DropperState.PRE_TRANSFER;
        driveGears = DriveGears.NOT_ENGAGED;
        intakeState = IntakeState.TUCK;
        driverCurrent = 0;
        intakeCurrent = 0;
        dropperCurrent = 0;
        this.isBlue = isBlue;
        this.isAuto = isAuto;
        visionIntakeHeading = Math.toRadians(0);
        voltage = 0;
        isCameraRunning = false;
        isCoarseCameraMode = false;
        isVisionAligning = false;
        isIntakeTracking = false;
        currentAutoState = "";
        previousAutoState = "";
        debugColor = Color.BLACK;
        runDistanceSensor = false;
        distanceSensorValue = -1;
        autoRemainingTime = -1;
        headingLockEnabled = false;
        limelightLastRobotCoords = new Waypoint(0, 0, 0);
        intakeSlidePosition = 0;
        absoluteBlockPosition = new AbsoluteBlockPosition();
    }

    /**
     * @return the current state of the dropper
     */
    public DropperState getDropperState() {
        return dropperState;
    }

    /**
     * Sets the current state of the dropper
     *
     * @param dropperState the state of the dropper
     */
    public void setDropperState(DropperState dropperState) {
        this.dropperState = dropperState;
    }

    /**
     * @return the current state of the dropper claw
     */
    public ClawState getDropperClawState() {
        return dropperClawState;
    }

    /**
     * Sets the current state of the dropper claw
     *
     * @param dropperClawState the state of the dropper claw
     */
    public void setDropperClawState(ClawState dropperClawState) {
        this.dropperClawState = dropperClawState;
    }

    /**
     * @return the current pitch of the dropper claw in degrees
     */
    public double getDropperClawRotation() {
        return dropperClawRotation;
    }

    /**
     * Sets the current pitch of the dropper claw
     *
     * @param dropperClawRotation the pitch of the dropper claw in degrees
     */
    public void setDropperClawRotation(double dropperClawRotation) {
        this.dropperClawRotation = dropperClawRotation;
    }

    /**
     * @return the current orientation of the dropper claw in degrees
     */
    public double getDropperClawPitch() {
        return dropperClawPitch;
    }

    /**
     * Sets the current orientation of the dropper claw
     *
     * @param dropperClawPitch the orientation of the dropper claw in degrees
     */
    public void setDropperClawPitch(double dropperClawPitch) {
        this.dropperClawPitch = dropperClawPitch;
    }

    /**
     * @return true if the vertical extension is extended, false otherwise
     */
    public boolean isVerticalExtended() {
        return isVerticalExtended;
    }

    /**
     * Sets the current state of the vertical extension
     *
     * @param verticalExtended is the vertical extension extended
     */
    public void setVerticalExtended(boolean verticalExtended) {
        isVerticalExtended = verticalExtended;
    }

    /**
     * @return true if the robot is ascending, false otherwise
     */
    public boolean getIsAscending() {
        return isAscending;
    }

    /**
     * Sets the current state of the robot's ascending
     *
     * @param isAscending is the robot ascending
     */
    public void setIsAscending(boolean isAscending) {
        this.isAscending = isAscending;
    }

    /**
     * @return the current position of the block the robot is holding
     */
    public RobotBlockPosition getBlockPosition() {
        return blockPosition;
    }

    /**
     * Sets the current position of the robot's block
     *
     * @param hasBlock the position of the robot's block
     */
    public void setBlockPosition(RobotBlockPosition hasBlock) {
        this.blockPosition = hasBlock;
    }

    /**
     * @return the current pitch of the robot's intake claw in degrees
     */
    public double getIntakeClawPitch() {
        return intakeClawPitch;
    }

    /**
     * Sets the current pitch of the robot's intake claw
     *
     * @param intakeClawPitch the pitch of the robot's intake claw in degrees
     */
    public void setIntakeClawPitch(double intakeClawPitch) {
        this.intakeClawPitch = intakeClawPitch;
    }

    /**
     * @return the current orientation of the robot's intake claw in degrees
     */
    public double getIntakeClawRotation() {
        return intakeClawRotation;
    }

    /**
     * Sets the current orientation of the robot's intake claw
     *
     * @param intakeClawRotation the orientation of the robot's intake claw in degrees
     */
    public void setIntakeClawRotation(double intakeClawRotation) {
        this.intakeClawRotation = intakeClawRotation;
    }

    /**
     * @return the current state of the robot's intake claw
     */
    public ClawState getIntakeClawState() {
        return intakeClawState;
    }

    /**
     * Sets the current state of the robot's intake claw
     *
     * @param intakeClawState the state of the robot's intake claw
     */
    public void setIntakeClawState(ClawState intakeClawState) {
        this.intakeClawState = intakeClawState;
    }

    /**
     * @return true if the robot's horizontal extension is extended, false otherwise
     */
    public boolean isHorizontalExtended() {
        return isHorizontalExtended;
    }

    /**
     * Sets the current state of the robot's horizontal extension
     *
     * @param horizontalExtended is the horizontal extension extended
     */
    public void setHorizontalExtended(boolean horizontalExtended) {
        this.isHorizontalExtended = horizontalExtended;
    }

    /**
     * @return the current fine lateral position of the block from the robot
     */
    public double getBlockLateralFine() {
        return blockLateralFine;
    }

    /**
     * Sets the current fine forward position of the block
     *
     * @param blockLateralFine the fine forward position of the block from the robot
     */
    public void setBlockLateralFine(double blockLateralFine) {
        this.blockLateralFine = blockLateralFine;
    }

    /**
     * @return the current fine forward position of the block from the robot
     */
    public double getBlockForwardFine() {
        return blockForwardFine;
    }

    /**
     * Sets the current fine lateral position of the block
     *
     * @param blockForwardFine the fine lateral position of the block from the robot
     */
    public void setBlockForwardFine(double blockForwardFine) {
        this.blockForwardFine = blockForwardFine;
    }

    /**
     * @return the current color of the block the intake is detecting
     */
    public BlockColor getIntakeBlockColor() {
        return intakeBlockColor;
    }

    /**
     * Sets the current color of the block the intake is detecting
     *
     * @param blockColor the color of the block
     */
    public void setIntakeBlockColor(BlockColor blockColor) {
        this.intakeBlockColor = blockColor;
    }

    /**
     * @return the current color of the block the dropper is detecting
     */
    public BlockColor getDropperBlockColor() {
        return dropperBlockColor;
    }

    /**
     * Sets the current color of the block the dropper is detecting
     *
     * @param blockColor the color of the block
     */
    public void setDropperBlockColor(BlockColor blockColor) {
        this.dropperBlockColor = blockColor;
    }

    /**
     * @return the current coarse lateral position of the block from the robot
     */
    public double getBlockForwardCoarse() {
        return blockForwardCoarse;
    }

    /**
     * Sets the current coarse lateral position of the block
     *
     * @param blockForwardCoarse the coarse lateral position of the block from the robot
     */
    public void setBlockForwardCoarse(double blockForwardCoarse) {
        this.blockForwardCoarse = blockForwardCoarse;
    }

    /**
     * @return the current orientation of the block in degrees
     */
    public double getBlockOrientation() {
        return blockOrientation;
    }

    /**
     * Sets the current orientation of the block
     *
     * @param blockOrientation the orientation of the block in degrees
     */
    public void setBlockOrientation(double blockOrientation) {
        this.blockOrientation = blockOrientation;
    }

    /**
     * @return the current detected fine block orientation in degrees
     */
    public double getDetectedFineBlockOrientation() {
        return detectedFineBlockOrientation;
    }

    /**
     * Sets the current detected fine block orientation
     *
     * @param detectedFineBlockOrientation the orientation of the detected block in degrees
     */
    public void setDetectedFineBlockOrientation(double detectedFineBlockOrientation) {
        this.detectedFineBlockOrientation = detectedFineBlockOrientation;
    }

    /**
     * @return the current coarse lateral position of the block from the robot
     */
    public double getBlockLateralCoarse() {
        return blockLateralCoarse;
    }

    /**
     * Sets the current coarse lateral position of the block
     *
     * @param blockLateralCoarse the coarse lateral position of the block from the robot
     */
    public void setBlockLateralCoarse(double blockLateralCoarse) {
        this.blockLateralCoarse = blockLateralCoarse;
    }

    /**
     * @return the current state of the coarse block detection
     */
    public BlockDetectionState getCoarseBlockDetectionState() {
        return coarseBlockDetectionState;
    }

    /**
     * Sets the current state of the coarse block detection
     *
     * @param coarseBlockDetectionState the state of coarse block detection
     */
    public void setCoarseBlockDetectionState(BlockDetectionState coarseBlockDetectionState) {
        this.coarseBlockDetectionState = coarseBlockDetectionState;
    }

    /**
     * @return the current state of the fine block detection
     */
    public BlockDetectionState getFineBlockDetectionState() {
        return fineBlockDetectionState;
    }

    /**
     * Sets the current state of the fine block detection
     *
     * @param fineBlockDetectionState the state of the fine block detection
     */
    public void setFineBlockDetectionState(BlockDetectionState fineBlockDetectionState) {
        this.fineBlockDetectionState = fineBlockDetectionState;
    }

    /**
     * @return the current color preference of the block
     */
    public BlockColorPreference getBlockColorPreference() {
        return blockColorPreference;
    }

    /**
     * Sets the current color preference of the block
     *
     * @param blockColorPreference the color preference of the block
     */
    public void setBlockColorPreference(BlockColorPreference blockColorPreference) {
        this.blockColorPreference = blockColorPreference;
    }

    /**
     * @return the current pose of the robot (Inches and Radians)
     */
    public Waypoint getRobotCurrentPose() {
        return robotCurrentPose;
    }

    /**
     * Sets the current pose of the robot (Inches and Radians)
     *
     * @param robotPose the current pose of the robot
     */
    public void setRobotPose(Waypoint robotPose) {
        robotCurrentPose = robotPose;
    }

    /**
     * @return the current velocity of the robot (Inches and Radians)
     */
    public Waypoint getRobotVelocity() {
        return robotVelocity;
    }

    /**
     * Sets the current velocity of the robot (Inches and Radians)
     *
     * @param robotVelocity the current velocity of the robot
     */
    public void setRobotVelocity(Waypoint robotVelocity) {
        this.robotVelocity = robotVelocity;
    }

    /**
     * @return the current gear of the robot
     */
    public DriveGears getCurrentGear() {
        return driveGears;
    }

    /**
     * Sets the current gear of the robot
     *
     * @param gear the gear of the robot
     */
    public void setCurrentGear(DriveGears gear) {
        this.driveGears = gear;
    }

    /**
     * @return the current state of the intake
     */
    public IntakeState getIntakeState() {
        return intakeState;
    }

    /**
     * Sets the current intake state of the robot
     *
     * @param intakeState the state of the intake
     */
    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;
    }

    /**
     * Sets a particular robot error to true
     *
     * @param error the error to set
     */
    public void setError(RobotError error) {
        this.robotError |= error.code;
    }

    /**
     * Clears a particular robot error
     *
     * @param error the error to clear
     */
    public void clearError(RobotError error) {
        this.robotError &= ~error.code;
    }

    /**
     * Checks if a particular robot error is set
     *
     * @param error the error to check
     * @return true if the error is set, false otherwise
     */
    public boolean hasError(RobotError error) {
        return (this.robotError & error.code) != 0;
    }

    /**
     * @return the current driver current
     */
    public double getDriverCurrent() {
        return driverCurrent;
    }

    /**
     * Sets the current driver current
     *
     * @param driverCurrent the current driver current
     */
    public void setDriverCurrent(double driverCurrent) {
        this.driverCurrent = driverCurrent;
    }

    /**
     * @return the current intake current
     */
    public double getIntakeCurrent() {
        return intakeCurrent;
    }

    /**
     * Sets the current intake current
     *
     * @param intakeCurrent the current intake current
     */
    public void setIntakeCurrent(double intakeCurrent) {
        this.intakeCurrent = intakeCurrent;
    }

    /**
     * @return the current dropper current
     */
    public double getDropperCurrent() {
        return dropperCurrent;
    }

    /**
     * Sets the current dropper current
     *
     * @param dropperCurrent the current dropper current
     */
    public void setDropperCurrent(double dropperCurrent) {
        this.dropperCurrent = dropperCurrent;
    }

    /**
     * @return if the manual intake is selected
     */
    public boolean isManualIntakeSelected() {
        return isManualIntakeSelected;
    }

    /**
     * Sets intake control to be manual or autonomous (with vision)
     *
     * @param manualIntakeSelected Whether the intake should be manual or not
     */
    public void setManualIntakeSelected(boolean manualIntakeSelected) {
        isManualIntakeSelected = manualIntakeSelected;
    }

    /**
     * Get the alliance color
     *
     * @return Is alliance blue?
     */
    public boolean isBlue() {
        return this.isBlue;
    }

    /**
     * Get the opmode mode
     *
     * @return Is mode auto?
     */
    public boolean isAuto() {
        return this.isAuto;
    }

    /**
     * @return the final pose of the robot in a trajectory
     */
    public Waypoint getRobotFinalPose() {
        return robotFinalPose;
    }

    /**
     * Sets the final pose of the robot in a trajectory
     *
     * @param robotFinalPose the final pose of the robot
     */
    public void setRobotFinalPose(Waypoint robotFinalPose) {
        this.robotFinalPose = robotFinalPose;
    }

    /**
     * @return The vision intake heading to use
     */
    public double getVisionIntakeHeading() {
        return visionIntakeHeading;
    }

    /**
     * Sets the vision intake heading to use
     *
     * @param visionIntakeHeading the supplier for the vision intake heading to use
     */
    public void setVisionIntakeHeading(double visionIntakeHeading) {
        this.visionIntakeHeading = visionIntakeHeading;
    }

    /**
     * @return the current voltage of the robot
     */
    public double getVoltage() {
        return voltage;
    }

    /**
     * Sets the current voltage of the robot
     *
     * @param voltage the current voltage of the robot
     */
    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    /**
     * @return if the small camera is running
     */
    public boolean isCameraRunning() {
        return isCameraRunning;
    }

    /**
     * Sets if the small camera is running
     *
     * @param cameraRunning whether the small camera is running
     */
    public void setCameraRunning(boolean cameraRunning) {
        isCameraRunning = cameraRunning;
    }

    /**
     * @return true if the small camera is in course camera mode, false if not
     */
    public boolean isCoarseCameraMode() {
        return isCoarseCameraMode;
    }

    /**
     * Sets the small camera mode
     *
     * @param courseCameraMode whether the small camera is in course or fine camera mode
     */
    public void setCoarseCameraMode(boolean courseCameraMode) {
        isCoarseCameraMode = courseCameraMode;
    }

    /**
     * @return whether the robot is aligning with vision
     */
    public boolean isVisionAligning() {
        return isVisionAligning;
    }

    /**
     * Sets whether the robot is aligning with vision to intake
     */
    public void setVisionAligning(boolean visionAligning) {
        isVisionAligning = visionAligning;
    }

    /**
     * @return the current state of the autonomous command
     */
    public String getCurrentAutoState() {
        return currentAutoState;
    }

    /**
     * Sets the current state of the autonomous command
     */
    public void setCurrentAutoState(String currentAutoState) {
        this.currentAutoState = currentAutoState;
    }

    /**
     * @return the previous state of the autonomous command
     */
    public String getPreviousAutoState() {
        return previousAutoState;
    }

    /**
     * Sets the previous state of the autonomous command
     */
    public void setPreviousAutoState(String previousAutoState) {
        this.previousAutoState = previousAutoState;
    }

    /**
     * @return the current debug color
     */
    public Color getDebugColor() {
        return debugColor;
    }

    /**
     * Sets the current debug color
     */
    public void setDebugColor(Color color) {
        this.debugColor = color;
    }

    /**
     * @return whether or not the intake is tracking
     */
    public boolean isIntakeTracking() {
        return isIntakeTracking;
    }

    /**
     * Sets whether or not the intake is tracking
     *
     * @param intakeTracking whether or not the intake is tracking
     */
    public void setIntakeTracking(boolean intakeTracking) {
        isIntakeTracking = intakeTracking;
    }

    /**
     * @return whether or not the distance sensor is running
     */
    public boolean isRunDistanceSensor() {
        return runDistanceSensor;
    }

    /**
     * sets whether or not the distance sensor is running
     *
     * @param runDistanceSensor whether or not the distance sensor is running
     */
    public void setRunDistanceSensor(boolean runDistanceSensor) {
        this.runDistanceSensor = runDistanceSensor;
    }

    /**
     * @return get the distance sensor value
     */
    public double getDistanceSensorValue() {
        return distanceSensorValue;
    }

    /**
     * sets the distance sensor value
     *
     * @param distanceSensorValue the distance sensor value
     */
    public void setDistanceSensorValue(double distanceSensorValue) {
        this.distanceSensorValue = distanceSensorValue;
    }

    /**
     * @return the amount of time remaining in the autonomous
     */
    public double getAutoRemainingTime() {
        return (double) autoRemainingTime;
    }

    /**
     * Sets the amount of time remaining in the autonomous
     *
     * @param autoRemainingTime the amount of time remaining in the autonomous
     */
    public void setAutoRemainingTime(double autoRemainingTime) {
        this.autoRemainingTime = autoRemainingTime;
    }

    /**
     * @return whether or not the break beam is enabled
     */
    public boolean isBreakBeamEnabled() {
        return isBreakBeamEnabled;
    }

    /**
     * Sets whether or not the break beam is enabled
     *
     * @param breakBeamEnabled whether or not the break beam is enabled
     */
    public void setBreakBeamEnabled(boolean breakBeamEnabled) {
        isBreakBeamEnabled = breakBeamEnabled;
    }


    /**
     * @return whether or not the heading lock is enabled
     */
    public boolean isHeadingLockEnabled() {
        return headingLockEnabled;
    }

    /**
     * Sets whether or not the heading lock beam is enabled
     *
     * @param headingLockEnabled whether or not the break beam is enabled
     */
    public void setHeadingLock(boolean headingLockEnabled) {
        this.headingLockEnabled = headingLockEnabled;
    }

    /**
     * Gets the object which contains the detected block's attributes
     *
     * @return the object which contains the detected block's attributes
     */
    public AbsoluteBlockPosition getAbsoluteBlockPosition() {
        return absoluteBlockPosition;
    }

    /**
     * Gets the current intake slide position (inches)
     *
     * @return the current intake slide position
     */
    public double getIntakeSlidePosition() {
        return intakeSlidePosition;
    }

    /**
     * Sets the current intake slide position (inches)
     *
     * @param intakeSlidePosition the current intake slide position
     */
    public void setIntakeSlidePosition(double intakeSlidePosition) {
        this.intakeSlidePosition = intakeSlidePosition;
    }
}
