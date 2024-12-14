package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

@TeleOp(name = "New Intake Test", group = "Test")
public class NewIntakeTestOpmode extends BaseOpMode {
    @Override
    public void initialize(){
        GamepadEx gamepad = new GamepadEx(gamepad1);

        RobotState robotState = new RobotState();
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);
        registerSubsystems(intake);

        Trigger intakeSlidesTrigger = new Trigger(() ->
                gamepad.getLeftY() != 0
        );
        intakeSlidesTrigger.whileActiveContinuous(() -> intake.moveSlidesRelative(
                gamepad.getLeftY() * 2));

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                () -> intake.setClawRotationRelative(0.5)
        );
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                () -> intake.setClawRotationRelative(-0.5)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(
                intake::closeClaw,
                intake::openClaw
        );

        Trigger leftTrigger = new Trigger(() ->
                gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0
        );
        leftTrigger.whileActiveContinuous(() -> intake.setRotationRelative(-0.5));
        Trigger rightTrigger = new Trigger(() ->
                gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0
        );
        rightTrigger.whileActiveContinuous(() -> intake.setRotationRelative(0.5));

        Trigger pitchTrigger = new Trigger(() ->
                gamepad.getRightY() != 0);
        pitchTrigger.whileActiveContinuous(() -> intake.setPitchRelative(gamepad.getRightY() * 0.5));
    }
}
