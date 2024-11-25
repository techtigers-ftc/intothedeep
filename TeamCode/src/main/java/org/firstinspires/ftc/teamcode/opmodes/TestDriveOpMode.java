package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import team.techtigers.base.BaseOpMode;

@TeleOp
@SuppressWarnings("unused")
public class TestDriveOpMode extends BaseOpMode {
    private DriveSubsystem drive;

    @Override
    public void initialize() {
        drive = new DriveSubsystem(hardwareMap);
        registerSubsystems(drive);
    }

    @Override
    public void update() {
        // Drive the robot with tele-op controls
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x);
    }
}
