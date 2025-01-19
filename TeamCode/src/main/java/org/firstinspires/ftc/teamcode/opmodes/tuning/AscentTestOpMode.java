package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.AscentCommand;
import org.firstinspires.ftc.teamcode.commands.ManualDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

@TeleOp
public class AscentTestOpMode extends BaseOpMode {
    private RobotState robotState;
    private AscentSubsystem ascent;

    @Override
    public void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        robotState = new RobotState(true, false);

        ascent = new AscentSubsystem(hardwareMap, robotState);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        DropperSubsystem dropper = new DropperSubsystem(hardwareMap, robotState);
        registerSubsystems(drive, ascent, dropper);

        // RUMBLE
        Gamepad.RumbleEffect ascentEngagementRumble =
                new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.5, 500)
                .build();

        Gamepad.RumbleEffect ascentDisengagementRumble =
                new Gamepad.RumbleEffect.Builder()
                        .addStep(0.8, 0.3, 200)
                        .addStep(0, 0, 100)
                        .addStep(0.3, 0.8, 200)
                        .addStep(0, 0, 100)
                        .addStep(0.8, 0.8, 200)
                        .build();

        // Ascent Trigger
        Trigger doubleDriverTouchpad =
                new Trigger(() -> gamepad1.touchpad_finger_2);
        doubleDriverTouchpad.toggleWhenActive(
                () -> {
                    ascent.engageAscent();
                    gamepad1.runRumbleEffect(ascentDisengagementRumble);
                },
                () -> {
//                    ascent.disengageAscent();
                    gamepad1.runRumbleEffect(ascentEngagementRumble);
                }
        );

        // Ascent and drive and dropper
        Trigger ascentTrigger = new Trigger(() -> robotState.getIsAscending());

        ManualDriveCommand manualDriveCommand = new ManualDriveCommand(drive, driverGamepad);
        ascentTrigger.negate().whileActiveOnce(manualDriveCommand);

        AscentCommand ascentCommand = new AscentCommand(ascent, driverGamepad);
        ascentTrigger.whileActiveContinuous(ascentCommand);

        Trigger dropperSlidesTrigger = new Trigger(() ->
                driverGamepad.getRightY() != 0
        );
        dropperSlidesTrigger.and(ascentTrigger.negate()).whileActiveContinuous(() ->
                dropper.moveSlidesRelative(
                        -driverGamepad.getRightY() * 2.5)
        );
    }

    @Override
    public void update() {
        telemetry.addData("Is Ascending", robotState.getIsAscending());
//        telemetry.addData("Dropper height", ascent.getCurrentSlidePositionInches());
    }
}
