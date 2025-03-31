package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperUndersideCarryWallAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperUndersideSlapAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeNoTransferAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

@TeleOp(name = "Dropper Underside Slap Test OpMode", group = "Test")
public class DropperUndersideSlapTestOpMode extends BaseOpMode {
    private RobotState robotState;
    private DropperSubsystem dropper;

    public void initialize() {
        robotState = new RobotState(false, false);
        GamepadEx driverGamepad = new GamepadEx(gamepad1);

        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap, robotState);
        SensorSubsystem sensor = new SensorSubsystem(hardwareMap, robotState);
        dropper = new DropperSubsystem(hardwareMap, robotState);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        registerSubsystems(odometry, sensor, dropper, drive);

        DropperUndersideCarryWallAction undersideCarryWallAction = new DropperUndersideCarryWallAction(dropper, robotState);
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(undersideCarryWallAction);

        DropperUndersideSlapAction undersideSlapAction = new DropperUndersideSlapAction(drive, dropper, robotState);
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(undersideSlapAction);

        DropperWallIntakeNoTransferAction wallIntakeNoTransferAction = new DropperWallIntakeNoTransferAction(dropper, robotState);
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(wallIntakeNoTransferAction);
    }

    public void update() {
        telemetry.addData("Dropper state: ", robotState.getDropperState());
        telemetry.addData("Dropper Slide POS", dropper.getCurrentSlidePositionInches());
    }
}
