package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autostates.DriveToGeneralDropState;
import org.firstinspires.ftc.teamcode.autostates.DropState;
import org.firstinspires.ftc.teamcode.autostates.EndState;
import org.firstinspires.ftc.teamcode.autostates.DriveToPreloadDropState;
import org.firstinspires.ftc.teamcode.autostates.IntakeSampleState;
import org.firstinspires.ftc.teamcode.autostates.DriveToIntakeState;
import org.firstinspires.ftc.teamcode.subsystems.AutoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import java.util.function.DoubleSupplier;

import team.techtigers.base.BaseOpMode;
import team.techtigers.base.statemachine.StateMachine;
import team.techtigers.core.paths.Waypoint;

@Autonomous
public class BasketAutoOpMode extends BaseOpMode {
    private RobotState robotState;

    private DoubleSupplier distToTarget(RobotState robotState, Waypoint target) {
        return () -> Math.min(Math.hypot(target.getX() - robotState.getRobotCurrentPose().getX(),
                target.getY() - robotState.getRobotCurrentPose().getY()) - 7, 19);
    }

    @Override
    public void initialize() {
        StateMachine<AutoState> stateMachine = new StateMachine<>();
        robotState = new RobotState();

        // Initialize subsystems
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap,
                robotState, new Waypoint(29.75, 7.25, Math.toRadians(90)));
        DropperSubsystem dropper = new DropperSubsystem(hardwareMap,
                robotState);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);

        // Creating states
        DriveToPreloadDropState drivePreloadToBasket = new DriveToPreloadDropState(
                "firstDriveToBasket",
                drive,
                dropper,
                robotState);
        DriveStateConfigurator.configPreloadDrop(drivePreloadToBasket);

        DropState dropSample = new DropState(
                "drop",
                dropper);

        DriveToIntakeState driveIntakeFirstSample = new DriveToIntakeState(
                "firstDriveToIntake",
                drive,
                dropper,
                robotState);
        DriveStateConfigurator.configFirstSampleIntake(driveIntakeFirstSample);

        IntakeSampleState intakeFirstSample = new IntakeSampleState("intakeFirst", intake,
                robotState,
                distToTarget(robotState, new Waypoint(22.5, 45)),
                () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading()));

        DriveToGeneralDropState driveBasketFirstSample = new DriveToGeneralDropState(
                "secondDriveToBasket",
                drive,
                dropper,
                intake,
                robotState);
        DriveStateConfigurator.configFirstSampleDrop(driveBasketFirstSample);

        DriveToIntakeState driveIntakeSecondSample = new DriveToIntakeState(
                "secondDriveToIntake",
                drive,
                dropper,
                robotState);
        DriveStateConfigurator.configSecondSampleIntake(driveIntakeSecondSample);

        IntakeSampleState intakeSecondSample = new IntakeSampleState(
                "intakeSecond",
                intake,
                robotState,
                distToTarget(robotState, new Waypoint(12.5, 45)),
                () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading()));

        DriveToGeneralDropState driveBasketSecondSample = new DriveToGeneralDropState(
                "thirdDriveToBasket",
                drive,
                dropper,
                intake,
                robotState);
        DriveStateConfigurator.configSecondSampleDrop(driveBasketSecondSample);

        EndState end = new EndState("end");

        // Create the state machine
        stateMachine
                .addState(drivePreloadToBasket)
                .addState(dropSample)
                .addState(driveIntakeFirstSample)
                .addState(intakeFirstSample)
                .addState(driveBasketFirstSample)
                .addState(driveIntakeSecondSample)
                .addState(intakeSecondSample)
                .addState(driveBasketSecondSample)
                .addState(end)

                .addTransition(drivePreloadToBasket, dropSample, AutoState.END_1)
                .addTransition(dropSample, driveIntakeFirstSample, AutoState.END_1)
                .addTransition(driveIntakeFirstSample, intakeFirstSample, AutoState.END_1)
                .addTransition(intakeFirstSample, driveBasketFirstSample, AutoState.END_1)
                .addTransition(driveBasketFirstSample, dropSample, AutoState.END_1)
                .addTransition(dropSample, driveIntakeSecondSample, AutoState.END_2)
                .addTransition(driveIntakeSecondSample, intakeSecondSample, AutoState.END_1)
                .addTransition(intakeSecondSample, driveBasketSecondSample, AutoState.END_1)
                .addTransition(driveBasketSecondSample, dropSample, AutoState.END_1)
                .addTransition(dropSample, end, AutoState.END_3)

                .setCurrentState(drivePreloadToBasket);


        // Register subsystems + Create state machine subsystem
        AutoSubsystem auto = new AutoSubsystem(stateMachine);
        registerSubsystems(auto, drive, odometry, dropper);
    }

    @Override
    public void update() {
        telemetry.addData("Current Pose", robotState.getRobotCurrentPose());
        telemetry.addData("Final Pose", robotState.getRobotFinalPose());
    }
}
