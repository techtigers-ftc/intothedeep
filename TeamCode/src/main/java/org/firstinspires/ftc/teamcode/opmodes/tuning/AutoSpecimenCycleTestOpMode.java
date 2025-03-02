package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.AutoSpecimenCycleAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

@TeleOp
public class AutoSpecimenCycleTestOpMode extends BaseOpMode {
    private RobotState robotState;

    @Override
    public void initialize() {
        robotState = new RobotState(false, false);
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        DropperSubsystem dropper = new DropperSubsystem(hardwareMap, robotState);
        SensorSubsystem sensor = new SensorSubsystem(hardwareMap, robotState);
        registerSubsystems(drive, dropper, sensor);

        AutoSpecimenCycleAction autoSpecimenCycle = new AutoSpecimenCycleAction(drive, dropper, robotState);
        driverGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(autoSpecimenCycle);
    }
}
