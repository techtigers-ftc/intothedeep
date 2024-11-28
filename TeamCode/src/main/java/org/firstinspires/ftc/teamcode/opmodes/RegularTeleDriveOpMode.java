package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.PedroManualDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

@TeleOp(name = "Regular TeleDrive OpMode", group = "TeleOp")
public class RegularTeleDriveOpMode extends BaseOpMode {

    @Override
    public void initialize() {
        RobotState robotState = new RobotState();
        GamepadEx driverGamepad = new GamepadEx(gamepad1);

        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        registerSubsystems(drive);

        PedroManualDriveCommand command = new PedroManualDriveCommand(drive, robotState, driverGamepad);
        drive.setDefaultCommand(command);
    }
}
