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
public class AutonomousRPL extends LinearOpMode {

    DcMotor l;
    DcMotor r;
    DcMotor lb;
    DcMotor rb;
    DcMotor fly;
    DcMotor loader;
    DcMotor intake;
    OpticalDistanceSensor eodsBack;
    OpticalDistanceSensor eodsFore;
    ColorSensor color_left;
    Servo button_left;
    Servo button_right;
    Servo wall_servo;
    Servo fly_servo;
    TouchSensor touch;
    final float wheelDiameter = 10f;
    final float PI = 3.1415f;
    final float THRESHOLD = 0.13f;

    @Override
    public void runOpMode() throws InterruptedException {
        //Retrieves hardware instances
        l = hardwareMap.dcMotor.get("l");
        r = hardwareMap.dcMotor.get("r");
        rb = hardwareMap.dcMotor.get("rb");
        lb = hardwareMap.dcMotor.get("lb");
        fly = hardwareMap.dcMotor.get("fly");

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
        fly_servo = hardwareMap.servo.get("sf");
        //Sets Initial Servo Positions
        wall_servo.setPosition(0.31);
        button_left.setPosition(0.70);
        button_right.setPosition(0.92 );
        color_left.enableLed(false);
        fly_servo.setPosition(0.1);
        waitForStart();
        while (opModeIsActive()) {
            /**
            while (opModeIsActive()) {
                telemetry.addLine("Fore: " + eodsFore.getLightDetected() + "\nBack: " + eodsBack.getLightDetected());
                telemetry.update();
                sleepOpMode(100);
            }**/
            //Press First Beacon
            this.pressBeacon();
            if (!opModeIsActive()) break;
            //Particle Shooting
            fly.setPower(-0.95);
            driveEncoder(-0.8, -73);
            stopDrive();
            fly_servo.setPosition(0.9);
             fly.setPower(0);
             driveEncoder(0.8, 58);
            fly_servo.setPosition(0.1);
            //Right turn
            turnEncoder(0.6, 90);
            stopDrive();
            driveEncoder(0.78, 30);
            //Press Second Beacon
            this.pressBeacon();
            //Park on ramp
            driveEncoder(-0.12, -12);
            turnEncoder(0.6, -90);
            int i = 0;
            while (opModeIsActive() && i < 2) {
                if(eodsFore.getLightDetected() > THRESHOLD) {
                    i++;
                    sleepOpMode(250);
                }
                drive(0.65);
            }
            while(opModeIsActive() && eodsFore.getLightDetected() < THRESHOLD) {
                drive(0.65);
            }
            stopDrive();
            driveEncoder(0.4, 15);
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
             break;
        }
        //Stops Robot
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
        int ticks = (int) (cm * 1120 * 1.35/(PI * wheelDiameter));
        telemetry.addLine("Ticks: " + ticks);
        telemetry.update();
        l.setTargetPosition(ticks + l.getCurrentPosition());
        r.setTargetPosition(ticks + r.getCurrentPosition());
        lb.setTargetPosition(ticks + lb.getCurrentPosition());
        rb.setTargetPosition(ticks + rb.getCurrentPosition());
        while (l.isBusy() || r.isBusy() || lb.isBusy() || rb.isBusy()) {
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
        int ticks = (int) (2 * 116 *  1120 * degrees / (PI * wheelDiameter * 360));
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
        wall_servo.setPosition(0.31);
        button_left.setPosition(0.70);
        button_right.setPosition(0.92);
        color_left.enableLed(false);
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Moves to Line from Start
        while (eodsFore.getLightDetected() < THRESHOLD && opModeIsActive()) { drive(0.2); }
        telemetry.addLine("First Sensor Passed");
        telemetry.update();
        if (!opModeIsActive()) return;
        while(eodsBack.getLightDetected() < THRESHOLD && opModeIsActive()) { drive(0.2); telemetry.addLine("Fore: " + eodsFore.getLightDetected() + "\nBack: " + eodsBack.getLightDetected());
            telemetry.update();}
        stopDrive();
        sleepOpMode(250);
        if (!opModeIsActive()) return;
        while(eodsBack.getLightDetected() > THRESHOLD && opModeIsActive()) { drive(0.17); }
        stopDrive();
        sleepOpMode(250);
        if (!opModeIsActive()) return;
        while(eodsBack.getLightDetected() < THRESHOLD && opModeIsActive()) { drive(-0.17); }
        stopDrive();
        sleepOpMode(250);
        if (!opModeIsActive()) return;
        while (eodsFore.getLightDetected() < THRESHOLD && opModeIsActive()) {
            this.setLeftPower(-0.32);
            this.setRightPower(0.32);
        }
        stopDrive();
        sleepOpMode(250);
        if (!opModeIsActive()) return;
        if (eodsFore.getLightDetected() < THRESHOLD) {
            while (eodsFore.getLightDetected() < THRESHOLD && opModeIsActive()) {
                this.setLeftPower(0.2);
                this.setRightPower(-0.2);
            }
            sleepOpMode(50);
            while (eodsFore.getLightDetected() > THRESHOLD && opModeIsActive()) {
                this.setLeftPower(0.2);
                this.setRightPower(-0.2);
            }
            sleepOpMode(50);
            while (eodsFore.getLightDetected() < THRESHOLD && opModeIsActive()) {
                this.setLeftPower(-0.2);
                this.setRightPower(0.2);
            }
        }
        else {
            while (eodsFore.getLightDetected() > THRESHOLD && opModeIsActive()) {
                this.setLeftPower(0.2);
                this.setRightPower(-0.2);
            }
            sleepOpMode(50);
            while (eodsFore.getLightDetected() < THRESHOLD && opModeIsActive()) {
                this.setLeftPower(-0.2);
                this.setRightPower(0.2);
            }
        }
        stopDrive();
        sleepOpMode(250);
        if (!opModeIsActive()) return;
        stopDrive();
        //Move to wall
        while (!touch.isPressed() && opModeIsActive()) { drive(0.2); }
        stopDrive();
        if (!opModeIsActive()) return;
        //Gets First's Beacon color, true if red, false if blue
        boolean colorFirstSide = color_left.red() > color_left.blue();
        sleepOpMode(50);
        //Drives back from Beacon
        drive(-0.12);
        sleepOpMode(600);
        if (!opModeIsActive()) return;
        stopDrive();
        //Retracts button
        wall_servo.setPosition(0.9);
        if (!opModeIsActive()) return;
        //Deploys pusher servos
        if (colorFirstSide) { button_left.setPosition(0.01);
        } else { button_right.setPosition(0.15); }
        //Waits for servos to move
        sleepOpMode(350);
        if (!opModeIsActive()) return;
        stopDrive();
        turnEncoder(-0.6, 18);
        //Drives forward and presses button
        drive(0.13);
        sleepOpMode(1025);
        button_left.setPosition(0.70);
        button_right.setPosition(0.92);
        if (!opModeIsActive()) return;
        stopDrive();
    }
}




