package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


/**
 * Created by alexbulanov on 12/19/16.
 */
    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Wiggle Auto")

public class AutonomousRPLWiggle extends LinearOpMode {


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
    final float wheelDiameter = 10f;
    final float PI = 3.1415f;
    final float THRESHOLD = 0.13f;


    @Override
    public void runOpMode() throws InterruptedException {
        l = hardwareMap.dcMotor.get("l");
        r = hardwareMap.dcMotor.get("r");
        rb = hardwareMap.dcMotor.get("rb");
        lb = hardwareMap.dcMotor.get("lb");
        button_left = hardwareMap.servo.get("bl");
        button_right = hardwareMap.servo.get("br");
        eodsFore = hardwareMap.opticalDistanceSensor.get("eodsF");
        eodsBack = hardwareMap.opticalDistanceSensor.get("eodsB");
        eodsBack.enableLed(true);
        eodsFore.enableLed(true);
        color_left = hardwareMap.colorSensor.get("cl");
        r.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
        touch = hardwareMap.touchSensor.get("t");
        wall_servo = hardwareMap.servo.get("ws");
        wall_servo.setPosition(0.31);
        button_left.setPosition(0.70);
        button_right.setPosition(0.92);
        waitForStart();
        while (opModeIsActive()) {
            //Sets Initial Servo Positions
            color_left.enableLed(false);
            //Moves to Line from Start
            while (eodsFore.getLightDetected() < THRESHOLD && opModeIsActive()) {
                drive(0.6);
            }
            while(eodsBack.getLightDetected() < THRESHOLD && opModeIsActive()) { drive(0.2); }
            stopDrive();
            sleepOpMode(100);
            if (!opModeIsActive()) break;
            //Turns left onto Line
            while (eodsFore.getLightDetected() < THRESHOLD && opModeIsActive()) {
                setLeftPower(0.4);
                setRightPower(-0.4);
            }
            if (!opModeIsActive()) break;
            stopDrive();
            //Wiggle Line-Follower
            while (!touch.isPressed()) {
                while (eodsFore.getLightDetected() < THRESHOLD && !touch.isPressed() && opModeIsActive()) {
                    setRightPower(0.52);
                }
                if (!opModeIsActive()) break;
                stopDrive();
                while (eodsFore.getLightDetected() > THRESHOLD && !touch.isPressed() && opModeIsActive()) {
                    setLeftPower(0.52);
                }
                if (!opModeIsActive()) break;
                stopDrive();
            }
            stopDrive();
            if (!opModeIsActive()) break;
            //Gets First's Beacon color, true if red, false if blue
            boolean colorFirstSide = color_left.red() > color_left.blue();
            //Drives back from Beacon
            drive(-0.32);
            sleepOpMode(600);
            if (!opModeIsActive()) break;
            stopDrive();
            //Retracts button
            wall_servo.setPosition(0.1);
            if (!opModeIsActive()) break;
            sleepOpMode(550);
            if (!opModeIsActive()) break;
            stopDrive();
            //Drives forward and presses button
            drive(0.33);
            sleepOpMode(900);
            if (!opModeIsActive()) break;
            stopDrive();
            break;
        }
        stopDrive();
        l.close();
        r.close();
        lb.close();
        rb.close();
        button_left.close();
        button_right.close();
        wall_servo.close();
        touch.close();
        eodsFore.close();
        color_left.close();
        stop();
    }


    public void drive(double power) {
        l.setPower(power);
        r.setPower(power);
        lb.setPower(power);
        rb.setPower(power);
    }

    public void stopDrive() {
        drive(0);
    }

    public void setLeftPower(double power) {
        l.setPower(power);
        lb.setPower(power);
    }

    public void setRightPower(double power) {
        r.setPower(power);
        rb.setPower(power);
    }

    public void sleepOpMode(double millTime) throws InterruptedException {
        double time = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() < time + millTime) {
            this.sleep(1);
        }
    }

    public boolean RightRedI() throws InterruptedException{
        if (color_left.red() < 5.1 && color_left.blue() < 5.1 && color_left.blue() != color_left.red()) {
            return color_left.red() > color_left.blue();
        }
        else {
            sleepOpMode(1);
            return RightRedI();
        }
    }

    public boolean RightRed() throws InterruptedException{
        boolean redI = RightRedI();
        sleepOpMode(150);
        if (redI == RightRedI()) {
            return redI;
        }
        else {
            sleepOpMode(150);
            telemetry.addLine("No Agreement");
            telemetry.update();
            return RightRed();
        }
    }
}




