package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.utils.enums.BlockColor;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

import team.techtigers.core.paths.Waypoint;
import team.techtigers.core.utils.GlobalState;

/**
 * Implementation of a global state for the robot
 */
public class RobotState extends GlobalState {
    private Waypoint robotCurrentPose;
    private Waypoint robotVelocity;
    private BlockColorPreference blockColorPreference;
    private BlockDetectionState blockDetectionState;
    private double blockLateralCoarse;
    private double blockOrientation;
    private double blockForwardCoarse;
    private BlockColor blockColor;
    private double blockForwardFine;
    private double blockLateralFine;
    private boolean isHorizontalExtended;
    private ClawState intakeClawState;
    private double intakeClawRotation;
    private double intakeClawPitch;
    private RobotBlockPosition blockPosition;
    private boolean isAscending;
    private boolean isVerticalExtended;
    private double dropperClawPitch;
    private double dropperClawRotation;
    private ClawState dropperClawState;
    private DropperState dropperState;
    private DriveGears driveGears;
    private IntakeState intakeState;
    private int robotError;

    /**
     * Initializes a new RobotState
     */
    public RobotState() {
        robotCurrentPose = new Waypoint(0, 0, 0);
        robotVelocity = new Waypoint(0, 0, 0);
        blockColorPreference = BlockColorPreference.ANY;
        blockDetectionState = BlockDetectionState.NOT_DETECTED;
        blockLateralCoarse = 0;
        blockOrientation = 0;
        blockForwardCoarse = 0;
        blockColor = BlockColor.NONE;
        blockForwardFine = 0;
        blockLateralFine = 0;
        isHorizontalExtended = false;
        intakeClawState = ClawState.OPEN;
        intakeClawRotation = 0;
        intakeClawPitch = 0;
        blockPosition = RobotBlockPosition.NONE;
        isAscending = false;
        isVerticalExtended = false;
        dropperClawPitch = 0;
        dropperClawRotation = 0;
        dropperClawState = ClawState.OPEN;
        dropperState = DropperState.TRANSFER;
        driveGears = DriveGears.NOT_ENGAGED;
        intakeState = IntakeState.TUCK;
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
     * @return the current color of the block in the robot
     */
    public BlockColor getBlockColor() {
        return blockColor;
    }

    /**
     * Sets the current color of the block in the robot
     *
     * @param blockColor the color of the block
     */
    public void setBlockColor(BlockColor blockColor) {
        this.blockColor = blockColor;
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
     * @return the current state of block detection
     */
    public BlockDetectionState getBlockDetectionState() {
        return blockDetectionState;
    }

    /**
     * Sets the current state of block detection
     *
     * @param blockDetectionState the state of block detection
     */
    public void setBlockDetectionState(BlockDetectionState blockDetectionState) {
        this.blockDetectionState = blockDetectionState;
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
}
