package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeToReadyToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTuckAction;
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
        IntakePrepareToPickupAction intakePrepareToPickupAction = new IntakePrepareToPickupAction(intakeSubsystem, robotState);
        IntakeReadyToPickupAction intakeReadyToPickupAction = new IntakeReadyToPickupAction(intakeSubsystem, robotState);
        IntakePrepareToTransferAction intakePrepareToTransferAction = new IntakePrepareToTransferAction(intakeSubsystem, robotState);
        IntakeToReadyToTransferAction intakeToReadyToTransferAction = new IntakeToReadyToTransferAction(intakeSubsystem, robotState);

        Trigger rightBumper = manipulatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        Trigger leftBumper = manipulatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);

        Trigger inTuck = new Trigger(() -> robotState.getIntakeState() == IntakeState.TUCK);
        Trigger inPrepareToIntake = new Trigger(() -> robotState.getIntakeState() == IntakeState.PREPARE_TO_PICKUP);
        Trigger inReadyToIntake = new Trigger(() -> robotState.getIntakeState() == IntakeState.READY_TO_PICKUP);
        Trigger inPrepareToTransfer = new Trigger(() -> robotState.getIntakeState() == IntakeState.PREPARE_TO_TRANSFER);
        Trigger inReadyToTransfer = new Trigger(() -> robotState.getIntakeState() == IntakeState.READY_TO_TRANSFER);

        leftBumper.and(inPrepareToIntake).whenActive(intakeTuckCommand);
        leftBumper.and(inReadyToIntake).whenActive(intakePrepareToPickupAction);
        leftBumper.and(inPrepareToTransfer).whenActive(intakePrepareToPickupAction);
        leftBumper.and(inReadyToTransfer).whenActive(intakePrepareToPickupAction);

        rightBumper.and(inTuck).whenActive(intakePrepareToPickupAction);
        rightBumper.and(inPrepareToIntake).whenActive(intakeReadyToPickupAction);
        rightBumper.and(inReadyToIntake).whenActive(intakePrepareToTransferAction);
        rightBumper.and(inPrepareToTransfer).whenActive(intakeToReadyToTransferAction);
        // TODO: CHANGE THIS LOGIC
        rightBumper.and(inReadyToTransfer).whenActive(intakeTuckCommand);



    }

    @Override
    public void update(){
        telemetry.addData("Has Intake Error?", robotState.hasError(RobotError.INVALID_INTAKE_POSITION));
        telemetry.addData("Claw rotation angle", intakeSubsystem.getClawRotation());
        telemetry.addData("Claw diff pitch", intakeSubsystem.getPitch());
        telemetry.addData("Claw diff rotation", intakeSubsystem.getRotation());
        telemetry.addData("Slides Position", intakeSubsystem.getCurrentSlidePositionInches());
        telemetry.addData("Current Intake State", robotState.getIntakeState());
    }
}
