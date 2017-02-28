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
    DcMotor fly;
    DcMotor intake;
    //Constants
    final float WHEEL_DIAMETER = 10f;
    final float PI = 3.1415f;
    final float THRESHOLD = 0.2f;
    final float LEFT_RETRACTED = 0.90f;
    final float LEFT_DEPLOYED = 0.01f;
    final float RIGHT_RETRACTED = 0.92f;
    final float RIGHT_DEPLOYED = 0.15f;
    final float WS_RETRACTED = 0.7f;
    final float WS_DEPLOYED = 0.31f;


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
            /**
            while(opModeIsActive()) {
                telemetry.addLine("Fore: " + eodsFore.getLightDetected() + "\nBack: " + eodsBack.getLightDetected());
                telemetry.update();
                sleepOpMode(5);
            }**/
            pressBeacon();
            if (!opModeIsActive()) break;
            //Drive back to shoot
            wall_servo.setPosition(WS_RETRACTED);
            fly.setPower(-0.95);
            driveEncoder(-0.8, -72);
            //INSERT SHOOTING CODE
            intake.setPower(-0.95);
            sleepOpMode(4000);
            fly.setPower(0);
            intake.setPower(0);
            if (!opModeIsActive()) break;
            //Drive forwards
            drive(0.65);
            sleepOpMode(850);
            stopDrive();
            if (!opModeIsActive()) break;
            //RIGHT TURN, 90 DEGREES
            setLeftPower(0.5);
            setRightPower(-0.5);
            sleepOpMode(1500);
            if (!opModeIsActive()) break;
            stopDrive();
            //SECOND BEACON
            drive(0.4);
            sleepOpMode(300);
            pressBeacon();
            //DRIVE BACK FROM BEACON
            drive(-0.6);
            sleepOpMode(500);
            stopDrive();
             //LEFT TURN, 90 DEGREES
             setLeftPower(-0.5);
             setRightPower(0.5);
             sleepOpMode(1500);
            if (!opModeIsActive()) break;
             stopDrive();
            //DRIVE UNTIL CORNER VORTEX
            while (eodsBack.getLightDetected() < 0.5 && opModeIsActive()) {
                drive(0.8);
                sleepOpMode(1);
            }
            while(eodsFore.getLightDetected() < THRESHOLD && opModeIsActive()) {
                drive(0.6);
                sleepOpMode(1);
            }
            if (!opModeIsActive()) break;
            //DRIVE ONTO CORNER VORTEX
            drive(0.7);
            sleepOpMode(700);
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
        wall_servo.setPosition(WS_DEPLOYED);
        button_left.setPosition(LEFT_RETRACTED);
        button_right.setPosition(RIGHT_RETRACTED);
        color_left.enableLed(false);
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Moves to Line from Start
        /**
        while (eodsFore.getLightDetected() < THRESHOLD && opModeIsActive()) {
            drive(0.2);
            telemetry.addLine("Fore: " + eodsFore.getLightDetected() + "\nBack: " + eodsBack.getLightDetected());
            telemetry.update();

        }**/
        while (eodsBack.getLightDetected() < 0.5 && opModeIsActive()) {
            drive(0.37);
            telemetry.addLine("Fore: " + eodsFore.getLightDetected() + "\nBack: " + eodsBack.getLightDetected());
            telemetry.update();
        }
        sleepOpMode(125);
        stopDrive();
        sleepOpMode(100);
        if (!opModeIsActive()) return;
        setLeftPower(-0.38);
        setRightPower(0.38);
        sleepOpMode(1600);
        //Turns left onto Line
        while (eodsFore.getLightDetected() < THRESHOLD && opModeIsActive())
            sleepOpMode(1);
        if (!opModeIsActive()) return;
        stopDrive();
        //Wiggle Line-Follower
        while (!touch.isPressed()) {
            while (eodsFore.getLightDetected() < THRESHOLD && !touch.isPressed() && opModeIsActive()) {
                setLeftPower(0.41);
            }
            if (!opModeIsActive()) return;
            stopDrive();
            while (eodsFore.getLightDetected() > THRESHOLD && !touch.isPressed() && opModeIsActive()) {
                setRightPower(0.41);
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
        if(colorFirstSide) {
            this.setLeftPower(0.2);
            this.setRightPower(-0.2);
        }
        //Waits for servos to move
        sleepOpMode(240);
        stopDrive();
        sleepOpMode(400);
        if (!opModeIsActive()) return;
        stopDrive();
        //Drives forward and presses button
        drive(0.17);
        sleepOpMode(1000);
        //Retracts Servos
        button_left.setPosition(LEFT_RETRACTED);
        button_right.setPosition(RIGHT_RETRACTED);
        stopDrive();
        return;
    }
}




