package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToFirstPush;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPreloadDropStateSpecimen;
import org.firstinspires.ftc.teamcode.autostates.specimen.DropSpecimenState;
import org.firstinspires.ftc.teamcode.autostates.specimen.EndState;
import org.firstinspires.ftc.teamcode.autostates.specimen.FirstPush;
import org.firstinspires.ftc.teamcode.subsystems.AutoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.BaseOpMode;
import team.techtigers.base.statemachine.StateMachine;
import team.techtigers.core.paths.Waypoint;

@Autonomous
public class SpecimenAutoOpMode extends BaseOpMode {
    private RobotState robotState;

    @Override
    public void initialize() {
        StateMachine<AutoState> stateMachine = new StateMachine<>();
        robotState = new RobotState(true, true);

        // Initialize subsystems
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap,
                robotState, new Waypoint(77, 7.25, Math.toRadians(90)));
        DropperSubsystem dropper = new DropperSubsystem(hardwareMap,
                robotState);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);

        // Creating states
        DriveToPreloadDropStateSpecimen driveChamberPreload = new DriveToPreloadDropStateSpecimen(
                "driveToChamberPreload",
                drive,
                dropper,
                robotState);
        SpecimenDriveStateConfigurator.configPreloadDrop(driveChamberPreload);

        DropSpecimenState dropSampleSpecimen = new DropSpecimenState(
                "drop",
                dropper,
                robotState);

        DriveToFirstPush driveToFirstPush = new DriveToFirstPush(
                "driveToFirstPush",
                drive,
                dropper,
                intake,
                robotState);
        SpecimenDriveStateConfigurator.configDriveToFirstPush(driveToFirstPush);

        FirstPush firstPush = new FirstPush(
                "firstPush",
                drive,
                robotState);
        SpecimenDriveStateConfigurator.configFirstPush(firstPush);

        EndState endState = new EndState(
                "end",
                dropper);


        // Create the state machine
        stateMachine
                .addState(driveChamberPreload)
                .addState(dropSampleSpecimen)
                .addState(driveToFirstPush)
                .addState(firstPush)
                .addState(endState)

                .addTransition(driveChamberPreload, dropSampleSpecimen, AutoState.DRIVE_END)
                .addTransition(dropSampleSpecimen, driveToFirstPush, AutoState.SPECIMEN_1_DROP_COMPLETE)
                .addTransition(driveToFirstPush, endState, AutoState.DRIVE_END)
//                .addTransition(firstPush, endState, AutoState.DRIVE_END)

                .setCurrentState(driveChamberPreload);


        // Register subsystems + Create state machine subsystem
        AutoSubsystem auto = new AutoSubsystem(stateMachine);
        registerSubsystems(auto, drive, odometry, dropper);
    }

    @Override
    public void update() {
        telemetry.addData("Current X", robotState.getRobotCurrentPose().getX());
        telemetry.addData("Current Y", robotState.getRobotCurrentPose().getY());
        telemetry.addData("Current Heading", robotState.getRobotCurrentPose().getHeading());
        telemetry.addLine();
        telemetry.addData("Expected X", robotState.getRobotFinalPose().getX());
        telemetry.addData("Expected Y", robotState.getRobotFinalPose().getY());
        telemetry.addData("Expected Heading", robotState.getRobotFinalPose().getHeading());
    }
}
