package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


/**
 * Created by alexbulanov on 12/19/16.
 */
    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red Auto Omni")
    @Disabled
    @Deprecated
public class AutonomousOmniRed extends LinearOpMode {


    DcMotor l;
    DcMotor r;
    DcMotor lb;
    DcMotor rb;
    OpticalDistanceSensor eodsBack;
    OpticalDistanceSensor eodsFore;
    ColorSensor color_left;
    Servo button_left;
    Servo button_right;
    Servo wall_servo;
    TouchSensor touch;

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
        color_left = hardwareMap.colorSensor.get("cl");
        l.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        touch = hardwareMap.touchSensor.get("t");
        wall_servo = hardwareMap.servo.get("ws");

        waitForStart();
        while (opModeIsActive()) {
            //Sets Initial Servo Positions
            wall_servo.setPosition(0.39);
            button_right.setPosition(0.1);
            button_left.setPosition(0.9);
            color_left.enableLed(false);
            //Moves to Line from Start
            while (eodsFore.getLightDetected() < 0.03 && opModeIsActive()) {
                drive(0.2);
            }
            if (!opModeIsActive()) break;
            while(eodsBack.getLightDetected() < 0.03 && opModeIsActive()) {
                drive(0.12);
            }
            if (!opModeIsActive()) break;
            while(eodsBack.getLightDetected() > 0.03 && opModeIsActive()) {
                drive(0.12);
            }
            stopDrive();
            while (eodsFore.getLightDetected() < 0.03 && opModeIsActive()) {
                lb.setPower(-0.15);
                rb.setPower(0.15);
            }
            if (!opModeIsActive()) break;
            stopDrive();
            //Wiggle Line-Follower
            while (!touch.isPressed() && opModeIsActive()) {
                drive(0.2);
            }
            stopDrive();
            if (!opModeIsActive()) break;
            //Gets First's Beacon color, true if red, false if blue
            boolean colorFirstSide = color_left.red() > color_left.blue();
            //Drives back from Beacon
            drive(-0.12);
            sleepOpMode(600);
            if (!opModeIsActive()) break;
            stopDrive();
            //Retracts button
            wall_servo.setPosition(0.1);
            if (!opModeIsActive()) break;
            //Deploys pusher servos
            if (colorFirstSide) {
                button_right.setPosition(0.95);
            } else {
                button_left.setPosition(0.05);
            }
            //Waits for servos to move
            sleepOpMode(550);
            if (!opModeIsActive()) break;
            stopDrive();
            //Drives forward and presses button
            drive(0.13);
            sleepOpMode(1525);
            if (!opModeIsActive()) break;
            stopDrive();
            //SECOND BEACON
            //Drives Back
            drive(-0.2);
            sleepOpMode(400);
            stopDrive();
            if (!opModeIsActive()) break;
            //Sets Servos
            wall_servo.setPosition(0.37);
            button_right.setPosition(0.1);
            button_left.setPosition(0.9);
            //Right turn
            setLeftPower(0.18);
            setRightPower(-0.18);
            sleepOpMode(1550);
            stopDrive();
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
        eodsBack.close();
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
}




