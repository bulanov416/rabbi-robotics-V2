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
    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red Auto Comp")
@Disabled
    @Deprecated
public class AutonomousRedCompetition extends LinearOpMode {


    DcMotor l;
    DcMotor r;
    DcMotor lb;
    DcMotor rb;
    OpticalDistanceSensor eods;
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
        eods = hardwareMap.opticalDistanceSensor.get("eodsF");
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
            while (eods.getLightDetected() < 0.03 && opModeIsActive()) {
                drive(0.6);
            }
            if (!opModeIsActive()) break;
            stopDrive();
            //Drives slightly past Line
            drive(0.6);
            sleepOpMode(100);
            stopDrive();
            //Turns left onto Line
            while (eods.getLightDetected() < 0.03 && opModeIsActive()) {
                setLeftPower(0.4);
                setRightPower(-0.4);
            }
            if (!opModeIsActive()) break;
            //Turns until past Line to follow Left edge of Line
            while (eods.getLightDetected() > 0.03 && opModeIsActive()) {
            }
            if (!opModeIsActive()) break;
            stopDrive();
            //Wiggle Line-Follower
            while (!touch.isPressed()) {
                while (eods.getLightDetected() < 0.03 && !touch.isPressed() && opModeIsActive()) {
                    setRightPower(0.34);
                }
                if (!opModeIsActive()) break;
                stopDrive();
                while (eods.getLightDetected() > 0.03 && !touch.isPressed() && opModeIsActive()) {
                    setLeftPower(0.32);
                }
                if (!opModeIsActive()) break;
                stopDrive();
            }
            stopDrive();
            if (!opModeIsActive()) break;
            //Gets First's Beacon color, true if red, false if blue
            boolean colorFirstSide = color_left.red() > color_left.blue();
            telemetry.addLine("Color Right First: " + Boolean.toString(colorFirstSide));
            telemetry.update();
            //Drives back from Beacon
            drive(-0.32);
            sleepOpMode(600);
            if (!opModeIsActive()) break;
            stopDrive();
            //Retracts button
            wall_servo.setPosition(0.1);
            if (!opModeIsActive()) break;
            /**
            //Deploys pusher servos
            if (!colorFirstSide) {
                button_right.setPosition(0.95);
            } else {
                button_left.setPosition(0.05);
            }**/
            //Waits for servos to move
            sleepOpMode(550);
            //Turns and moves forward until crosses left edge of Line
            /*
            while (eodsFore.getLightDetected() > 0.03 && opModeIsActive()) {
                setRightPower(0.12);
            }
            */
            if (!opModeIsActive()) break;
            stopDrive();
            //Drives forward and presses button
            drive(0.33);
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
            setLeftPower(-0.18);
            setRightPower(0.18);
            sleepOpMode(1550);
            if (!opModeIsActive()) break;
            //Drives until second Line
            while (eods.getLightDetected() < 0.03 && opModeIsActive()) {
                drive(0.15);
            }
            if (!opModeIsActive()) break;
            stopDrive();
            //Drives slightly past Line
            drive(0.2);
            sleepOpMode(100);
            stopDrive();
            //Turns left until encounters Line
            while (eods.getLightDetected() < 0.03 && opModeIsActive()) {
                setLeftPower(0.18);
                setRightPower(-0.18);
            }
            if (!opModeIsActive()) break;
            //Continues until passes Line
            while (eods.getLightDetected() > 0.03 && opModeIsActive()) {
            }
            if (!opModeIsActive()) break;
            stopDrive();
            //Wiggle Line-Follower
            while (!touch.isPressed()) {
                while (eods.getLightDetected() < 0.03 && !touch.isPressed() && opModeIsActive()) {
                    setRightPower(0.17);
                }
                if (!opModeIsActive()) break;
                stopDrive();
                while (eods.getLightDetected() > 0.03 && !touch.isPressed() && opModeIsActive()) {
                    setLeftPower(0.12);
                }
                if (!opModeIsActive()) break;
                stopDrive();
            }
            if (!opModeIsActive()) break;
            //Gets Second Beacon right color, true if red
            boolean colorSecondSide = color_left.red() > color_left.blue();
            telemetry.addLine("Color Right Second: " + Boolean.toString(colorFirstSide));
            telemetry.update();
            stopDrive();
            //Drives Back
            drive(-0.12);
            sleepOpMode(600);
            /*
            //Left Turn, minor
            setRightPower(-0.17);
            setLeftPower(0.17);
            sleepOpModeSided(350);
            */
            stopDrive();
            if (!opModeIsActive()) break;
            stopDrive();
            //Retracts wall servo
            wall_servo.setPosition(0.1);
            if (!opModeIsActive()) break;
            //Deploys button pushers
            if (!colorSecondSide) {
                button_right.setPosition(0.95);
            } else {
                button_left.setPosition(0.05);
            }
            //Waits for Servos to deploy
            sleepOpMode(550);
            //Final adjustments, depending on position relative to line
            /*
            if (eodsFore.getLightDetected() > 0.03) {
                while (eodsFore.getLightDetected() > 0.03 && opModeIsActive()) {
                    setLeftPower(0.17);
                }
            }
            else {
                while (eodsFore.getLightDetected() < 0.03 && opModeIsActive()) {
                    setRightPower(0.17);
                }
            }
            */
            if (!opModeIsActive()) break;
            stopDrive();
            //Drives forward, presses button
            drive(0.13);
            sleepOpMode(1620);
            stopDrive();
            //Drives back
            drive(-0.13);
            sleepOpMode(400);
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
        eods.close();
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




