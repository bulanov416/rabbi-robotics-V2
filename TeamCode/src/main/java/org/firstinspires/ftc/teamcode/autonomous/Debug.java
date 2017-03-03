package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Levi on 2/27/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Debug")

public class Debug extends LinearOpMode{

    DcMotor l;
    DcMotor r;
    DcMotor lb;
    DcMotor rb;
    OpticalDistanceSensor eodsFore;
    OpticalDistanceSensor eodsBack;
    ColorSensor color_left;
    Servo button_left;
    Servo button_right;
    Servo wall_servo;
    TouchSensor touch;
    //Constants
    final float WHEEL_DIAMETER = 10f;
    final float PI = 3.1415f;
    final float THRESHOLD = 0.3f;
    final float LEFT_RETRACTED = 0.90f;
    final float LEFT_DEPLOYED = 0.01f;
    final float RIGHT_RETRACTED = 0.92f;
    final float RIGHT_DEPLOYED = 0.15f;
    final float WS_RETRACTED = 0.7f;
    final float WS_DEPLOYED = 0.2f;

    @Override
    public void runOpMode() throws InterruptedException {
        //Motors
        l = hardwareMap.dcMotor.get("l");
        r = hardwareMap.dcMotor.get("r");
        rb = hardwareMap.dcMotor.get("rb");
        lb = hardwareMap.dcMotor.get("lb");
        r.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
        //Servos
        button_left = hardwareMap.servo.get("bl");
        button_right = hardwareMap.servo.get("br");
        wall_servo = hardwareMap.servo.get("wsl");
        wall_servo.setPosition(WS_DEPLOYED);
        button_left.setPosition(LEFT_RETRACTED);
        button_right.setPosition(RIGHT_RETRACTED);
        //Sensors
        eodsFore = hardwareMap.opticalDistanceSensor.get("eodsF");
        eodsBack = hardwareMap.opticalDistanceSensor.get("eodsB");
        eodsBack.enableLed(true);
        eodsFore.enableLed(true);
        color_left = hardwareMap.colorSensor.get("cl");
        touch = hardwareMap.touchSensor.get("tr");
        waitForStart();
        while (opModeIsActive()) {
            //Press First Beacon
            while (opModeIsActive()) {
                telemetry.addLine("Fore: " + eodsFore.getLightDetected() + "\nBack: " + eodsBack.getLightDetected());
                telemetry.addLine("\nTouch Sensor: " + (touch.isPressed() ? "true" : "false"));
                telemetry.update();
                sleepOpMode(5);
            }
        }
    }

    public void sleepOpMode(double millTime) throws InterruptedException {
        double time = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() < time + millTime) {
            this.sleep(1);
        }
    }

}
