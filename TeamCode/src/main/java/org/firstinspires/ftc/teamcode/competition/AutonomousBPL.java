package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


/**
 * Created by alexbulanov on 12/19/16.
 */
    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue Beacon Push")

public class AutonomousBPL extends LinearOpMode {


    DcMotor l;
    DcMotor r;
    DcMotor lb;
    DcMotor rb;
    OpticalDistanceSensor eodsFore;
    OpticalDistanceSensor eodsBack;
    ColorSensor color_left;
    Servo button_left;
    Servo button_right;
    Servo wall_servo_right;
    Servo wall_servo_left;
    TouchSensor touch;
    DcMotor fly;
    DcMotor intake;
    //Constants
    final float WHEEL_DIAMETER = 10f;
    final float PI = 3.1415f;
    final float THRESHOLD = 0.17f;
    final float LEFT_RETRACTED = 0.90f;
    final float LEFT_DEPLOYED = 0.01f;
    final float RIGHT_RETRACTED = 0.92f;
    final float RIGHT_DEPLOYED = 0.15f;
    final float WSR_RETRACTED = 0.25f;
    final float WSR_DEPLOYED = 0.83f;
    final float WSL_RETRACTED = 0.8f;
    final float WSL_DEPLOYED = 0.28f;



    @Override
    public void runOpMode() throws InterruptedException {
        //Motors
        l = hardwareMap.dcMotor.get("l");
        r = hardwareMap.dcMotor.get("r");
        rb = hardwareMap.dcMotor.get("rb");
        lb = hardwareMap.dcMotor.get("lb");
        r.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
        fly = hardwareMap.dcMotor.get("fly");
        intake = hardwareMap.dcMotor.get("in");
        //Servos
        button_left = hardwareMap.servo.get("bl");
        button_right = hardwareMap.servo.get("br");
        wall_servo_right = hardwareMap.servo.get("wsr");
        wall_servo_right.setPosition(WSR_RETRACTED);
        wall_servo_left = hardwareMap.servo.get("wsl");
        wall_servo_left.setPosition(WSL_RETRACTED);
        button_left.setPosition(LEFT_RETRACTED);
        button_right.setPosition(RIGHT_RETRACTED);
        //Sensors
        eodsFore = hardwareMap.opticalDistanceSensor.get("eodsF");
        eodsBack = hardwareMap.opticalDistanceSensor.get("eodsB");
        eodsBack.enableLed(true);
        eodsFore.enableLed(true);
        color_left = hardwareMap.colorSensor.get("cl");
        color_left.enableLed(false);
        touch = hardwareMap.touchSensor.get("tr");
        waitForStart();
        while (opModeIsActive()) {
            //Press First Beacon
            drive(1);
            setLeftPower(0.8);
            sleepOpMode(1000);
            pressBeacon();
            if (!opModeIsActive()) break;
            //Drive back to shoot
            wall_servo_right.setPosition(WSR_RETRACTED);
            fly.setPower(-1);
            driveEncoder(-0.8, -80);
            //SHOOTING
            intake.setPower(-0.95);
            sleepOpMode(3600);
            fly.setPower(-0.7);
            intake.setPower(0);
            if (!opModeIsActive()) break;
            drive(-1);
            sleepOpMode(240);
            //Drive forwards
            drive(0.8);
            sleepOpMode(1320);
            fly.setPower(-0.4);
            stopDrive();
            if (!opModeIsActive()) break;
            //LEFT TURN, 90 DEGREES
            setLeftPower(-1);
            setRightPower(1);
            sleepOpMode(600);
            fly.setPower(-0.2);
            sleepOpMode(330);
            fly.setPower(0);
            if (!opModeIsActive()) break;
            stopDrive();
            //SECOND BEACON
            drive(1);
            sleepOpMode(280);
            pressBeacon();
            //DRIVE BACK FROM BEACON
            drive(-1);
            sleepOpMode(600);
            stopDrive();
             //RIGHT TURN, 90 DEGREES
             setLeftPower(1);
             setRightPower(-1);
             sleepOpMode(980);
            if (!opModeIsActive()) break;
             stopDrive();
            //DRIVE ONTO CORNER VORTEX
            drive(1);
            sleepOpMode(4500);
            stopDrive();
            break;
        }
        //Closes hardware instances
        stopDrive();
        l.close();
        r.close();
        lb.close();
        rb.close();
        fly.close();
        intake.close();
        button_left.close();
        button_right.close();
        wall_servo_right.close();
        wall_servo_left.close();
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
        r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        l.setPower(power);
        r.setPower(power);
        lb.setPower(0);
        rb.setPower(0);
        int ticks = (int) (cm * 1120 * 1.35/(PI * WHEEL_DIAMETER));
        l.setTargetPosition(ticks + l.getCurrentPosition());
        r.setTargetPosition(ticks + r.getCurrentPosition());
        long startTime = System.currentTimeMillis();
        while ((l.isBusy() || r.isBusy()) && opModeIsActive() && System.currentTimeMillis() < (startTime + 3500)) {
            sleepOpMode(1);
        }
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopDrive();
    }
    //Degrees clockwise, power absolute value
    public void turnEncoder(double power, double degrees) throws InterruptedException{
        long startTime = System.currentTimeMillis();
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
        while ((l.isBusy() || r.isBusy() || lb.isBusy() || rb.isBusy()) && opModeIsActive() && System.currentTimeMillis() < startTime + 3000) {
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

    public void print(String input) {
        telemetry.addLine(input);
        telemetry.update();
    }

    public void pressBeacon() throws InterruptedException{
        //Sets Initial Servo Positions
        wall_servo_right.setPosition(WSR_RETRACTED);
        button_left.setPosition(LEFT_RETRACTED);
        button_right.setPosition(RIGHT_RETRACTED);
        color_left.enableLed(false);
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Moves to Line from Start
        while (eodsBack.getLightDetected() < 0.5 && opModeIsActive()) {
            drive(0.37);
        }
        stopDrive();
        sleepOpMode(30);
        while(eodsBack.getLightDetected() < 0.5 && opModeIsActive()) {
            drive(-0.2);
        }
        stopDrive();
        sleepOpMode(100);
        if (!opModeIsActive()) return;
        setLeftPower(0.38);
        setRightPower(-0.38);
        sleepOpMode(500);
        //Turns right onto Line
        while (eodsFore.getLightDetected() < THRESHOLD && opModeIsActive()) sleepOpMode(1);
        sleepOpMode(5);
        if (!opModeIsActive()) return;
        stopDrive();
        //Ensure on LEFT side of line
        int counter = 0;
        boolean passed = false;
        while(counter < 500 && opModeIsActive()) {
            if(eodsFore.getLightDetected() < THRESHOLD) {
                passed = true;
                break;
            }
            sleepOpMode(1);
            counter++;
        }
        if (passed) {
            sleepOpMode(100);
            setLeftPower(-0.44);
            while(eodsFore.getLightDetected() < THRESHOLD && opModeIsActive()) sleepOpMode(1);
            sleepOpMode(5);
            while(eodsFore.getLightDetected() > THRESHOLD && opModeIsActive()) sleepOpMode(1);
        }
        stopDrive();
        sleepOpMode(150);
        wall_servo_right.setPosition(WSR_DEPLOYED);
        sleepOpMode(200);
        //Wiggle Line-Follower
        while (!touch.isPressed()) {
            while (eodsFore.getLightDetected() < THRESHOLD && !touch.isPressed() && opModeIsActive()) {
                setLeftPower(0.32);
            }
            if (!opModeIsActive()) return;
            stopDrive();
            while (eodsFore.getLightDetected() > THRESHOLD && !touch.isPressed() && opModeIsActive()) {
                setRightPower(0.32);
            }
            if (!opModeIsActive()) return;
            stopDrive();
            sleepOpMode(50);
        }
        stopDrive();
        if (!opModeIsActive()) return;
        sleepOpMode(50);
        //Gets First's Beacon color, true if red, false if blue
        boolean colorFirstSide = color_left.red() < color_left.blue();
        stopDrive();
        if(colorFirstSide) {
            setRightPower(-0.4);
            sleepOpMode(120);
        }
        stopDrive();
        //Drives back from Beacon
        drive(-1);
        sleepOpMode(120);
        if (!opModeIsActive()) return;
        stopDrive();
        //Retracts button
        wall_servo_right.setPosition(WSR_RETRACTED);
        if (!opModeIsActive()) return;
        //Deploys pusher servos
        if (colorFirstSide) { button_left.setPosition(LEFT_DEPLOYED);
        } else { button_right.setPosition(RIGHT_DEPLOYED); }
        //Waits for servos to move
        stopDrive();
        sleepOpMode(400);
        if (!opModeIsActive()) return;
        stopDrive();
        //Drives forward and presses button
        drive(0.31);
        sleepOpMode(1000);
        //Retracts Servos
        button_left.setPosition(LEFT_RETRACTED);
        button_right.setPosition(RIGHT_RETRACTED);
        stopDrive();
        return;
    }
}




