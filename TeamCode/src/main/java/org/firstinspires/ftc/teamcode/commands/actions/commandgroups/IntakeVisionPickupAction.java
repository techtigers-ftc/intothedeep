package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.drive.TeleHoldPointAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

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
     * @param gamepad         the driver gamepad, which is rumbled during a
     *                        drive takeover. If a null gamepad is passed in,
     *                        nothing will rumble
     */
    public IntakeVisionPickupAction(IntakeSubsystem intake,
                                    DropperSubsystem dropper,
                                    DriveSubsystem drive,
                                    RobotState robotState,
                                    DoubleSupplier headingSupplier,
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
                        new IntakePrepareToPickupAction(intake, dropper, robotState, robotState::getBlockForwardCoarse),
                        new TeleHoldPointAction(drive, robotState,
                                () -> robotState.getRobotCurrentPose().getX() + Math.sin(robotState.getRobotCurrentPose().getHeading()) * robotState.getBlockLateralCoarse(),
                                () -> robotState.getRobotCurrentPose().getY() - Math.cos(robotState.getRobotCurrentPose().getHeading()) * robotState.getBlockLateralCoarse() + 1,
                                headingSupplier, 0.3, Math.toRadians(2)
                        )
                ),
                new IntakeReadyToPickupAction(intake, robotState, IntakeSubsystem.CLAW_ROTATION_PICKUP_POSITION)
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
        this(intake, dropper, drive, robotState, () -> heading, gamepad);
    }
}
