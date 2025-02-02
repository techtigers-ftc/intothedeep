package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFullReadyToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.drive.TeleHoldPointAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeCheckSensorAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeLoosenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import java.util.function.DoubleSupplier;

/**
 * A command group that automatically picks up a specimen using the vision system and limelight
 */
public class IntakeVisionPickupAction extends SequentialCommandGroup {

    /**
     * Creates a new IntakeVisionPickupAction
     *
     * @param intake          the intake subsystem
     * @param dropper         the dropper subsystem
     * @param drive           the drive subsystem
     * @param robotState      the robot state
     * @param headingSupplier the heading supplier that supplier the heading values to the target position
     * @param clawRotationSupplier the claw rotation supplier that supplies the claw rotation values to the target position
     * @param gamepad         the driver gamepad, which is rumbled during a
     *                        drive takeover. If a null gamepad is passed in,
     *                        nothing will rumble
     */
    public IntakeVisionPickupAction(IntakeSubsystem intake,
                                    DropperSubsystem dropper,
                                    DriveSubsystem drive,
                                    RobotState robotState,
                                    DoubleSupplier headingSupplier,
                                    DoubleSupplier clawRotationSupplier,
                                    GamepadEx gamepad) {
        addRequirements(intake, dropper, drive);
        addCommands(
                // Aligns the robot to the heading given by the heading supplier
                new TeleHoldPointAction(drive, robotState,
                        () -> robotState.getRobotCurrentPose().getX(),
                        () -> robotState.getRobotCurrentPose().getY(),
                        headingSupplier,
                        0.3, Math.toRadians(2)
                ),
                // Moves intake to prepare to pickup and runs intake and drive coarse align
                new ParallelCommandGroup(
                        new IntakeReadyToPickupAction(intake, robotState, robotState::getBlockForwardCoarse, clawRotationSupplier),
                        new TeleHoldPointAction(drive, robotState,
                                () -> robotState.getRobotCurrentPose().getX() + Math.sin(robotState.getRobotCurrentPose().getHeading()) * (robotState.getBlockLateralCoarse()),
                                () -> robotState.getRobotCurrentPose().getY() - Math.cos(robotState.getRobotCurrentPose().getHeading()) * (robotState.getBlockLateralCoarse()),
                                headingSupplier, 0.3, Math.toRadians(2)
                        )
                ),
                new IntakeFullReadyToTransferAction(intake, dropper, robotState, this)
        );
    }

    /**
     * Overload constructor for intake vision pickup action
     *
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param drive      the drive subsystem
     * @param robotState the robot state
     * @param heading    the heading value
     * @param gamepad    the driver gamepad. If a null gamepad is passed in,
     *                   nothing will rumble
     */
    public IntakeVisionPickupAction(IntakeSubsystem intake,
                                    DropperSubsystem dropper,
                                    DriveSubsystem drive,
                                    RobotState robotState, double heading,
                                    GamepadEx gamepad) {
        this(intake, dropper, drive, robotState, () -> heading, robotState::getBlockOrientation, gamepad);
    }

    /**
     * Yet another overload constructor for intake vision pickup action without given heading
     *
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param drive      the drive subsystem
     * @param robotState the robot state
     * @param gamepad    the driver gamepad. If a null gamepad is passed in,
     *                   nothing will rumble
     */
    public IntakeVisionPickupAction(IntakeSubsystem intake,
                                    DropperSubsystem dropper,
                                    DriveSubsystem drive,
                                    RobotState robotState,
                                    GamepadEx gamepad) {
        this(intake, dropper, drive, robotState, () -> robotState.getRobotCurrentPose().getHeading(), robotState::getBlockOrientation,gamepad);
    }
}
