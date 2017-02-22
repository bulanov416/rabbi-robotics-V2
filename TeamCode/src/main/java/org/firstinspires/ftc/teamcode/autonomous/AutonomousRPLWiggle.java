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
    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RPL")

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
    //Constants
    final float WHEEL_DIAMETER = 10f;
    final float PI = 3.1415f;
    final float THRESHOLD = 0.13f;
    final float LEFT_RETRACTED = 0.90f;
    final float LEFT_DEPLOYED = 0.01f;
    final float RIGHT_RETRACTED = 0.92f;
    final float RIGHT_DEPLOYED = 0.15f;
    final float WS_RETRACTED = 0.0f;
    final float WS_DEPLOYED = 0.98f;


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
        wall_servo = hardwareMap.servo.get("ws");
        wall_servo.setPosition(WS_DEPLOYED);
        button_left.setPosition(LEFT_RETRACTED);
        button_right.setPosition(RIGHT_RETRACTED);
        //Sensors
        eodsFore = hardwareMap.opticalDistanceSensor.get("eodsF");
        eodsBack = hardwareMap.opticalDistanceSensor.get("eodsB");
        eodsBack.enableLed(true);
        eodsFore.enableLed(true);
        color_left = hardwareMap.colorSensor.get("cl");
        touch = hardwareMap.touchSensor.get("t");
        waitForStart();
        while (opModeIsActive()) {
            //Press First Beacon
            pressBeacon();
            if (!opModeIsActive()) break;
            //Drive back to shoot
            driveEncoder(-0.8, -73);
            //INSERT SHOOTING CODE

            //Drive forwards
            driveEncoder(0.65, 58);
            //RIGHT TURN
            //SECOND BEACON
            //pressBeacon();
            //LEFT TURN, 135 DEGREES
            //DRIVE UNTIL CENTER VORTEX
            /**
            drive(0.5);
            while (eodsFore.getLightDetected() < THRESHOLD) {
                sleepOpMode(1);
            }
            //DRIVE ONTO CENTER VORTEX
            /**
            drive(0.3);
            sleepOpMode(350);**/

        }
        //Closes hardware instances
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

    public void setMotorModes(DcMotor.RunMode runMode) {
        lb.setMode(runMode);
        rb.setMode(runMode);
        r.setMode(runMode);
        l.setMode(runMode);
    }
    //CM forward, power forwards
    public void driveEncoder(double power, double cm) throws InterruptedException{
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        l.setPower(power);
        r.setPower(power);
        lb.setPower(power);
        rb.setPower(power);
        int ticks = (int) (cm * 1120 * 1.35/(PI * WHEEL_DIAMETER));
        telemetry.addLine("Ticks: " + ticks);
        telemetry.update();
        l.setTargetPosition(ticks + l.getCurrentPosition());
        r.setTargetPosition(ticks + r.getCurrentPosition());
        lb.setTargetPosition(ticks + lb.getCurrentPosition());
        rb.setTargetPosition(ticks + rb.getCurrentPosition());
        while ((l.isBusy() || r.isBusy() || lb.isBusy() || rb.isBusy()) && opModeIsActive()) {
            sleepOpMode(1);
        }
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stopDrive();
    }
    //Degrees clockwise, power absolute value
    public void turnEncoder(double power, double degrees) throws InterruptedException{
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        int ticks = (int) (2.05 * 116 *  1120 * degrees / (PI * WHEEL_DIAMETER * 360));
        if (degrees > 0) {
            l.setPower(power);
            r.setPower(-power);
            lb.setPower(power);
            rb.setPower(-power);
            l.setTargetPosition(ticks + l.getCurrentPosition());
            r.setTargetPosition(-ticks + r.getCurrentPosition());
            lb.setTargetPosition(ticks + lb.getCurrentPosition());
            rb.setTargetPosition(-ticks + rb.getCurrentPosition());
        }
        else {
            l.setPower(-power);
            r.setPower(power);
            lb.setPower(-power);
            rb.setPower(power);
            l.setTargetPosition(-ticks  + l.getCurrentPosition());
            r.setTargetPosition(ticks + r.getCurrentPosition());
            lb.setTargetPosition(-ticks + lb.getCurrentPosition());
            rb.setTargetPosition(ticks + rb.getCurrentPosition());
        }
        while ((l.isBusy() || r.isBusy() || lb.isBusy() || rb.isBusy())&&opModeIsActive()) {
            sleepOpMode(1);
        }
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopDrive() {
        drive(0);
    }

    public void setLeftPower(double power) {
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l.setPower(power);
        lb.setPower(power);
    }

    public void setRightPower(double power) {
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.setPower(power);
        rb.setPower(power);
    }

    public void sleepOpMode(double millTime) throws InterruptedException {
        double time = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() < time + millTime) {
            this.sleep(1);
        }
    }

    public void pressBeacon() throws InterruptedException{
        //Sets Initial Servo Positions
        wall_servo.setPosition(WS_DEPLOYED);
        button_left.setPosition(LEFT_RETRACTED);
        button_right.setPosition(RIGHT_RETRACTED);
        color_left.enableLed(false);
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Moves to Line from Start
        while (eodsFore.getLightDetected() < THRESHOLD && opModeIsActive()) drive(0.6);
        while (eodsBack.getLightDetected() < THRESHOLD && opModeIsActive()) drive(0.2);
        stopDrive();
        sleepOpMode(100);
        if (!opModeIsActive()) return;
        //Turns left onto Line
        while (eodsFore.getLightDetected() < THRESHOLD && opModeIsActive()) {
            setLeftPower(-0.4);
            setRightPower(0.4);
        }
        if (!opModeIsActive()) return;
        stopDrive();
        //Wiggle Line-Follower
        while (!touch.isPressed()) {
            while (eodsFore.getLightDetected() < THRESHOLD && !touch.isPressed() && opModeIsActive()) {
                setRightPower(0.52);
            }
            if (!opModeIsActive()) return;
            stopDrive();
            while (eodsFore.getLightDetected() > THRESHOLD && !touch.isPressed() && opModeIsActive()) {
                setLeftPower(0.52);
            }
            if (!opModeIsActive()) return;
            stopDrive();
        }
        stopDrive();
        if (!opModeIsActive()) return;
        sleepOpMode(50);
        //Gets First's Beacon color, true if red, false if blue
        boolean colorFirstSide = color_left.red() > color_left.blue();
        //Drives back from Beacon
        drive(-0.12);
        sleepOpMode(500);
        if (!opModeIsActive()) return;
        stopDrive();
        //Retracts button
        wall_servo.setPosition(WS_RETRACTED);
        if (!opModeIsActive()) return;
        //Deploys pusher servos
        if (colorFirstSide) { button_left.setPosition(LEFT_DEPLOYED);
        } else { button_right.setPosition(RIGHT_DEPLOYED); }
        //Final Correction
        if(!colorFirstSide) {
            this.setLeftPower(-0.2);
            this.setRightPower(0.2);
        }
        //Waits for servos to move
        sleepOpMode(250);
        if (!opModeIsActive()) return;
        stopDrive();
        //Drives forward and presses button
        drive(0.13);
        sleepOpMode(900);
        //Retracts Servos
        button_left.setPosition(LEFT_RETRACTED);
        button_right.setPosition(RIGHT_RETRACTED);
        stopDrive();
        return;
    }
}




