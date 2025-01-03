package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autostates.DropState;
import org.firstinspires.ftc.teamcode.autostates.EndState;
import org.firstinspires.ftc.teamcode.autostates.FirstDriveToBasketState;
import org.firstinspires.ftc.teamcode.autostates.FirstIntakeState;
import org.firstinspires.ftc.teamcode.autostates.FirstDriveToIntakeState;
import org.firstinspires.ftc.teamcode.autostates.SecondDriveToBasketState;
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
public class BasketAutoOpMode extends BaseOpMode {
    private RobotState robotState;

    @Override
    public void initialize() {
        StateMachine stateMachine = new StateMachine();
        robotState = new RobotState();

        // Initialize subsystems
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap,
                robotState, new Waypoint(29.75, 7.25, Math.toRadians(90)));
        DropperSubsystem dropper = new DropperSubsystem(hardwareMap,
                robotState);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);

        // Create the state machine
        stateMachine
                .addState(new FirstDriveToBasketState("firstDrive", drive,
                        dropper, robotState))
                .addState(new DropState("drop", dropper, robotState))
                .addState(new FirstDriveToIntakeState("secondDrive", drive, dropper,
                        robotState))
                .addState(new FirstIntakeState("intakefirstpreload", intake, robotState))
                .addState(new SecondDriveToBasketState("second-drive", drive, dropper, intake, robotState))
                .addState(new EndState("end"))

                .from("firstDrive")
                .to("drop")
                .when(AutoState.END_1)
                .from("drop")
                .to("secondDrive")
                .when(AutoState.END_1)
                .from("secondDrive")
                .to("intakefirstpreload")
                .when(AutoState.END_1)
                .from("intakefirstpreload")
                .to("second-drive")
                .when(AutoState.END_1)
                .from("second-drive")
                .to("drop")
                .when(AutoState.END_1)
                .from("drop")
                .to("end")
                .when(AutoState.END_2)



                .setFirstState("firstDrive");


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
