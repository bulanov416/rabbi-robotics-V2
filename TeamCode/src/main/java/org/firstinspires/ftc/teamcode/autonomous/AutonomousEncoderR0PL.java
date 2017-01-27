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
    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "R0PL")
public class AutonomousEncoderR0PL extends LinearOpMode {


    DcMotor l;
    DcMotor r;
    DcMotor lb;
    DcMotor rb;
    DcMotor fly;
    OpticalDistanceSensor eodsBack;
    OpticalDistanceSensor eodsFore;
    ColorSensor color_left;
    ColorSensor color_down;
    Servo button_left;
    Servo button_right;
    Servo wall_servo;
    Servo fly_servo;
    TouchSensor touch;
    final float wheelDiameter = 10f;
    final float pi = 3.1415f;

    @Override
    public void runOpMode() throws InterruptedException {
        l = hardwareMap.dcMotor.get("l");
        r = hardwareMap.dcMotor.get("r");
        rb = hardwareMap.dcMotor.get("rb");
        lb = hardwareMap.dcMotor.get("lb");
        fly = hardwareMap.dcMotor.get("fly");

        button_left = hardwareMap.servo.get("bl");
        button_right = hardwareMap.servo.get("br");
        eodsFore = hardwareMap.opticalDistanceSensor.get("eodsF");
        eodsBack = hardwareMap.opticalDistanceSensor.get("eodsB");
        color_left = hardwareMap.colorSensor.get("cl");
        color_down = hardwareMap.colorSensor.get("cd");
        l.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        touch = hardwareMap.touchSensor.get("t");
        wall_servo = hardwareMap.servo.get("ws");
        fly_servo = hardwareMap.servo.get("sf");
        waitForStart();
        while (opModeIsActive()) {
            //Sets Initial Servo Positions
            wall_servo.setPosition(0.39);
            button_right.setPosition(0.1);
            button_left.setPosition(0.9);
            color_left.enableLed(false);
            color_down.enableLed(true);
            fly_servo.setPosition(0.1);
            //Press First Beacon
            this.pressBeacon();
            if (!opModeIsActive()) break;
            //Particle Shooting
            fly.setPower(-0.95);
            driveEncoder(-0.3, -50);
            fly_servo.setPosition(0.9);
            driveEncoder(0.3, 50);
            fly.setPower(0);
            fly_servo.setPosition(0.1);
            //Right turn
            turnEncoder(0.2, 90);
            stopDrive();
            //Press Second Beacon
            this.pressBeacon();
            //Park on ramp
            driveEncoder(-0.12, -12);
            turnEncoder(0.2, -90);
            drive(0.2);
            while (color_down.red() <= 3) {
                sleepOpMode(1);
            }
            stopDrive();
            driveEncoder(0.3, 15);
            //Park on central vortex
            /*
             driveEncoder(-0.12, -12);
            turnEncoder(0.2, -135);
            drive(0.2);
            while (color_down.red() <= 3) {
                sleepOpMode(1);
            }
            driveEncoder(0.3, 8);
            * */
        }
        stopDrive();
        l.close();
        r.close();
        lb.close();
        rb.close();
        fly.close();
        button_left.close();
        button_right.close();
        wall_servo.close();
        fly_servo.close();
        touch.close();
        eodsFore.close();
        eodsBack.close();
        color_left.close();
        color_down.close();
        stop();
    }


    public void drive(double power) {
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l.setPower(power);
        r.setPower(power);
        lb.setPower(power);
        rb.setPower(power);
    }

    public void setMotorModes(DcMotor.RunMode runMode) {
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(runMode);
        rb.setMode(runMode);
        r.setMode(runMode);
        l.setMode(runMode);
    }
//CM forward, power forwards
    public void driveEncoder(double power, double cm) throws InterruptedException{
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
        l.setPower(power);
        r.setPower(power);
        lb.setPower(power);
        rb.setPower(power);
        int ticks = (int) (cm * 1120 /(pi * wheelDiameter));
        l.setTargetPosition(ticks);
        r.setTargetPosition(ticks);
        lb.setTargetPosition(ticks);
        rb.setTargetPosition(ticks);
        while (l.isBusy() || r.isBusy() || lb.isBusy() || rb.isBusy()) {
            sleepOpMode(1);
        }
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
//Degrees clockwise, power absolute value
    public void turnEncoder(double power, double degrees) throws InterruptedException{
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
        int ticks = (int) (116 *  1120 * degrees / (pi * wheelDiameter * 360));
        if (degrees > 0) {
            l.setPower(power);
            r.setPower(-power);
            lb.setPower(power);
            rb.setPower(-power);
            l.setTargetPosition(ticks);
            r.setTargetPosition(-ticks);
            lb.setTargetPosition(ticks);
            rb.setTargetPosition(-ticks);
        }
        else {
            l.setPower(-power);
            r.setPower(power);
            lb.setPower(-power);
            rb.setPower(power);
            l.setTargetPosition(-ticks);
            r.setTargetPosition(ticks);
            lb.setTargetPosition(-ticks);
            rb.setTargetPosition(ticks);
        }
        while (l.isBusy() || r.isBusy() || lb.isBusy() || rb.isBusy()) {
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
        wall_servo.setPosition(0.39);
        button_right.setPosition(0.1);
        button_left.setPosition(0.9);
        color_left.enableLed(false);
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Moves to Line from Start
        while (eodsFore.getLightDetected() < 0.03 && opModeIsActive()) { drive(0.2); }
        if (!opModeIsActive()) return;
        while(eodsBack.getLightDetected() < 0.03 && opModeIsActive()) { drive(0.12); }
        if (!opModeIsActive()) return;
        while(eodsBack.getLightDetected() > 0.03 && opModeIsActive()) { drive(0.12); }
        stopDrive();
        while (eodsFore.getLightDetected() < 0.03 && opModeIsActive()) {
            this.setLeftPower(-0.15);
            this.setRightPower(0.15);
        }
        if (!opModeIsActive()) return;
        stopDrive();
        //Move to wall
        while (!touch.isPressed() && opModeIsActive()) { drive(0.2); }
        stopDrive();
        if (!opModeIsActive()) return;
        //Gets First's Beacon color, true if red, false if blue
        boolean colorFirstSide = color_left.red() > color_left.blue();
        //Drives back from Beacon
        drive(-0.12);
        sleepOpMode(600);
        if (!opModeIsActive()) return;
        stopDrive();
        //Retracts button
        wall_servo.setPosition(0.1);
        if (!opModeIsActive()) return;
        //Deploys pusher servos
        if (colorFirstSide) { button_right.setPosition(0.95);
        } else { button_left.setPosition(0.01); }
        //Waits for servos to move
        sleepOpMode(350);
        if (!opModeIsActive()) return;
        stopDrive();
        //Drives forward and presses button
        drive(0.13);
        sleepOpMode(1525);
        if (!opModeIsActive()) return;
        stopDrive();
        //Drives Back
        drive(-0.2);
        sleepOpMode(400);
        stopDrive();
        if (!opModeIsActive()) return;
        //Sets Servos
        wall_servo.setPosition(0.37);
        button_right.setPosition(0.1);
        button_left.setPosition(0.9);
        sleepOpMode(350);
    }
}




