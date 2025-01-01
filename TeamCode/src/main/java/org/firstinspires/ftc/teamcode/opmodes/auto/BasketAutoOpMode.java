package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autostates.EndState;
import org.firstinspires.ftc.teamcode.autostates.FirstDriveToBasketState;
import org.firstinspires.ftc.teamcode.subsystems.AutoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
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

        // Create the state machine
        stateMachine
                .addState(new FirstDriveToBasketState("testDrive", drive, robotState))
                .addState(new EndState("end"))

                .from("testDrive")
                .to("end")
                .when(AutoState.END_1)

                .setFirstState("testDrive");


        // Register subsystems + Create state machine subsystem
        AutoSubsystem auto = new AutoSubsystem(stateMachine);
        registerSubsystems(auto, drive, odometry);
    }

    @Override
    public void update() {
        telemetry.addData("Current Pose", robotState.getRobotCurrentPose());
    }
}
