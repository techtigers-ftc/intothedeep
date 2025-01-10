package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.autocommands.NewAutoDriveCommand;

import team.techtigers.base.BaseOpMode;

@TeleOp
public class AutoDriveTestOpMode extends BaseOpMode {
    private NewAutoDriveCommand command;

    @Override
    public void initialize() {
        command = new NewAutoDriveCommand(hardwareMap);
    }

    @Override
    public void justAfterStart() {
        command.schedule();
    }

    @Override
    public void update() {
        telemetry.addData("Pose: ", command.follower.getPose());
    }
}
