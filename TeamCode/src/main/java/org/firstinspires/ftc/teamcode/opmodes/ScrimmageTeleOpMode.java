package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.IntakeToIntakeCommandGroup;
import org.firstinspires.ftc.teamcode.commands.IntakeToTransferCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

@TeleOp(name = "Scrimmage TeleOp Mode", group = "Scrimmage")
public class ScrimmageTeleOpMode extends BaseOpMode {
    @Override
    public void initialize() {
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        RobotState robotState = new RobotState();
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);
        registerSubsystems(intake);

        IntakeToIntakeCommandGroup intakeToIntake = new IntakeToIntakeCommandGroup(intake);
        IntakeToTransferCommandGroup intakeToTransfer = new IntakeToTransferCommandGroup(intake);

        gamepadEx.getGamepadButton(GamepadKeys.Button.X).toggleWhenPressed(
                () -> intake.setWristAbsolute(180,90),
                () -> intake.setWristAbsolute(180,0)
        );

        gamepadEx.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(
                intake::closeClaw,
                intake::openClaw
        );

        Trigger slidesTrigger = new Trigger(() ->
                gamepadEx.getLeftY() != 0
        );

        slidesTrigger.whileActiveContinuous(() -> intake.moveSlidesRelative(gamepadEx.getLeftY() * 2));

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(intakeToTransfer);

        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(intakeToIntake);
    }
}
