package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperBackSlapAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperBackwardCarryAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperForwardCarryAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperFrontSlapAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperHighBasketAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperPreTransferAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

import team.techtigers.base.BaseOpMode;

@TeleOp(name = "Dropper State Test OpMode", group = "Test")
public class DropperStateTestOpmode extends BaseOpMode {

    private RobotState robotState;
    private DropperSubsystem dropper;
    private IntakeSubsystem intake;

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        GamepadEx manipulatorGamepad = new GamepadEx(gamepad2);
        robotState = new RobotState();
        dropper = new DropperSubsystem(hardwareMap, robotState);
        intake = new IntakeSubsystem(hardwareMap, robotState);
        registerSubsystems(dropper, intake);

        DropperBackSlapAction dropperBackSlapAction = new DropperBackSlapAction(dropper);
        DropperFrontSlapAction dropperFrontSlapAction = new DropperFrontSlapAction(dropper);
        DropperBackwardCarryAction dropperBackwardCarryAction = new DropperBackwardCarryAction(dropper, intake, robotState);
        DropperForwardCarryAction dropperForwardCarryAction = new DropperForwardCarryAction(dropper, intake, robotState);
        DropperHighBasketAction dropperHighBasketAction = new DropperHighBasketAction(dropper, intake, robotState);
        DropperPreTransferAction dropperPreTransferAction = new DropperPreTransferAction(dropper, robotState);

        Trigger dpadLeft = manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        Trigger dpadRight = manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);

        Trigger forwardCarry = new Trigger(() -> robotState.getDropperState() == DropperState.FORWARD_CARRY);
        Trigger backwardCarry = new Trigger(() -> robotState.getDropperState() == DropperState.BACKWARD_CARRY);

        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(dropperPreTransferAction);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(dropperHighBasketAction);
        dpadLeft.and(backwardCarry).whenActive(dropperBackSlapAction);
        dpadLeft.and(backwardCarry.negate()).whenActive(dropperBackwardCarryAction);
        dpadRight.and(forwardCarry).whenActive(dropperFrontSlapAction);
        dpadRight.and(forwardCarry.negate()).whenActive(dropperForwardCarryAction);

    }
    @Override
    public void update() {
        double currentPos = dropper.getCurrentSlidePositionInches();
        double expectedPos = dropper.getTargetPositionInches();
        double error = expectedPos - currentPos;

        telemetry.addData("CurrentPosInches", currentPos);
        telemetry.addData("ExpectedPosInches", expectedPos);
        telemetry.addData("Error", error);
    }
}
