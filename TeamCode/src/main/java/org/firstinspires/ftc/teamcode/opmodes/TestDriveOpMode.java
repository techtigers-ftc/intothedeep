package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.PedroManualDriveCommand;

@TeleOp
@SuppressWarnings("unused")
public class TestDriveOpMode extends ConfigOpModeTele {

    @Override
    public void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx manipulatorGamepad = new GamepadEx(gamepad2);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        registerSubsystems(drive);

        PedroManualDriveCommand manualDriveCommand = new PedroManualDriveCommand(drive, robotState, follower, driverGamepad);
    }

    @Override
    public void update() {

    }
}
