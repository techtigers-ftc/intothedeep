package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.IntakePrepareToIntakeAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

import team.techtigers.base.BaseOpMode;

/**
 * OpMode to test the different states of the intake claw
 */
@TeleOp(name = "Intake State Test OpMode", group = "Test")
public class IntakeStateTestOpMode extends BaseOpMode {
    private IntakeSubsystem intakeSubsystem;
    private RobotState robotState;

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        GamepadEx manipulatorGamepad = new GamepadEx(gamepad2);
        robotState = new RobotState();
        intakeSubsystem = new IntakeSubsystem(hardwareMap, robotState);
        registerSubsystems(intakeSubsystem);

        IntakeTuckAction intakeTuckCommand = new IntakeTuckAction(intakeSubsystem, robotState);
        IntakePrepareToIntakeAction intakePrepareToIntakeAction = new IntakePrepareToIntakeAction(intakeSubsystem, robotState);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(intakeTuckCommand);

        Trigger rightBumper = manipulatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        Trigger leftBumper = manipulatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);

        Trigger inTuck = new Trigger(() -> robotState.getIntakeState() == IntakeState.TUCK);
        Trigger inPrepareToIntake = new Trigger(() -> robotState.getIntakeState() == IntakeState.PREPARE_TO_INTAKE);
        Trigger inReadyToIntake = new Trigger(() -> robotState.getIntakeState() == IntakeState.READY_TO_INTAKE);
        Trigger inPrepareToTransfer = new Trigger(() -> robotState.getIntakeState() == IntakeState.PREPARE_TO_TRANSFER);
        Trigger inReadyToTransfer = new Trigger(() -> robotState.getIntakeState() == IntakeState.READY_TO_TRANSFER);

        // To Tuck
        leftBumper.and(inPrepareToIntake).whenActive(intakeTuckCommand);
        // To Prepare to Intake
        rightBumper.and(inTuck).whenActive(intakePrepareToIntakeAction);


    }

    @Override
    public void update(){
        telemetry.addData("Has Intake Error?", robotState.hasError(RobotError.INVALID_INTAKE_POSITION));
        telemetry.addData("Claw rotation angle", intakeSubsystem.getClawRotation());
        telemetry.addData("Claw diff pitch", intakeSubsystem.getPitch());
        telemetry.addData("Claw diff rotation", intakeSubsystem.getRotation());
        telemetry.addData("Slides Position", intakeSubsystem.getCurrentSlidePositionInches());
    }
}
