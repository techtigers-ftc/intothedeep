package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autostates.DropState;
import org.firstinspires.ftc.teamcode.autostates.EndState;
import org.firstinspires.ftc.teamcode.autostates.FirstDriveToBasketState;
import org.firstinspires.ftc.teamcode.autostates.IntakeSampleState;
import org.firstinspires.ftc.teamcode.autostates.FirstDriveToIntakeState;
import org.firstinspires.ftc.teamcode.autostates.SecondDriveToBasketState;
import org.firstinspires.ftc.teamcode.autostates.SecondDriveToIntakeState;
import org.firstinspires.ftc.teamcode.autostates.ThirdDriveToBasketState;
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
                .addState(new FirstDriveToBasketState("firstDriveToBasket", drive,
                        dropper, robotState))
                .addState(new DropState("drop", dropper))
                .addState(new FirstDriveToIntakeState("firstDriveToIntake", drive, dropper,
                        robotState))
                .addState(new IntakeSampleState("intakeFirst", intake, robotState, distToTarget(
                        robotState, new Waypoint(22.5, 45)),
                        () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading())))
                .addState(new SecondDriveToBasketState("secondDriveToBasket", drive, dropper, intake, robotState))
                .addState(new SecondDriveToIntakeState("secondDriveToIntake", drive, dropper, robotState))
                .addState(new IntakeSampleState("intakeSecond", intake, robotState, distToTarget(
                        robotState, new Waypoint(12.5, 45)),
                        () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading())))
                .addState(new ThirdDriveToBasketState("thirdDriveToBasket", drive, dropper, intake, robotState))
                .addState(new EndState("end"))

                .from("firstDriveToBasket")
                .to("drop")
                .when(AutoState.END_1)
                
                .from("drop")
                .to("firstDriveToIntake")
                .when(AutoState.END_1)
                .from("firstDriveToIntake")
                .to("intakeFirst")
                .when(AutoState.END_1)
                .from("intakeFirst")
                .to("secondDriveToBasket")
                .when(AutoState.END_1)
                .from("secondDriveToBasket")
                .to("drop")
                .when(AutoState.END_1)
                
                .from("drop")
                .to("secondDriveToIntake")
                .when(AutoState.END_2)
                .from("secondDriveToIntake")
                .to("intakeSecond")
                .when(AutoState.END_1)
                .from("intakeSecond")
                .to("thirdDriveToBasket")
                .when(AutoState.END_1)
                .from("thirdDriveToBasket")
                .to("drop")
                .when(AutoState.END_1)
                .from("drop")
                .to("end")
                .when(AutoState.END_3)



                .setFirstState("firstDriveToBasket");


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
