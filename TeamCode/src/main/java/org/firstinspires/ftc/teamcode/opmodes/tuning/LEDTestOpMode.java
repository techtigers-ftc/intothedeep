package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;

@TeleOp
public class LEDTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
//        LED led = hardwareMap.get(LED.class, "intake_leds");
//        led.on();
//
        DigitalChannel led = hardwareMap.get(DigitalChannel.class,
                "intake_leds");
        led.setMode(DigitalChannel.Mode.OUTPUT);
        led.setState(true);

        waitForStart();

        while (opModeIsActive()) {
//            sleep(1000);
//            led.off();
            telemetry.addData("LED On?", led.getState());
            telemetry.update();
//            sleep(1000);
        }
    }
}
