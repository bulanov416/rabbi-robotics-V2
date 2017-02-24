package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Nathan on 1/26/17.
 *
 * This is not a Linear OpMode. Don't freak out though - look at the docs, and read the class. It's very easy
 * to wrap your head around - instead of one big method that runs from start to finish, there's a few small methods
 * run in a certain order.
 */
@TeleOp(name = "Drive Iterative")
@Disabled
public class DriveIterative extends OpMode {

    DcMotor l;
    DcMotor r;
    DcMotor lb;
    DcMotor rb;
    DcMotor lift;
    DcMotor fly;
    Servo button_left;
    Servo button_right;
    Servo wall_servo;
    TouchSensor touch;
    OpticalDistanceSensor eodsBack;
    OpticalDistanceSensor eodsFore;
    ColorSensor color_left;

    float gp1_left_stick_y;
    float left_drive_power;
    float gp1_right_stick_y;
    float right_drive_power;

    boolean autoOn = false;

    @Override
    public void init() {
        // this method runs once when the INIT button is pressed
        // Motors
        l = hardwareMap.dcMotor.get("l"); // AndyMark
        r = hardwareMap.dcMotor.get("r"); // AndyMark
        rb = hardwareMap.dcMotor.get("rb"); // AndyMark
        lb = hardwareMap.dcMotor.get("lb"); // AndyMark
        lift = hardwareMap.dcMotor.get("lift");
        fly = hardwareMap.dcMotor.get("fly");
        /// makes sure the motors run in the right direction
        l.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        // Servos
        button_left = hardwareMap.servo.get("bl");
        button_right = hardwareMap.servo.get("br");
        // Sensors - all currently disabled
        //touch = hardwareMap.touchSensor.get("t");
        //wall_servo = hardwareMap.servo.get("ws");
        //eodsFore = hardwareMap.opticalDistanceSensor.get("eodsF");
        //eodsBack = hardwareMap.opticalDistanceSensor.get("eodsB");
        //color_left = hardwareMap.colorSensor.get("cl");

        telemetry.addData("hardware: ", "Hardware Devices Initialized");
    }

    public void init_loop() {
        // this method loops until the PLAY button is pressed
    }

    @Override
    public void loop() {
        gp1_left_stick_y = Range.clip(gamepad1.left_stick_y, -1, 1); // update the left joystick value
        left_drive_power = gp1_left_stick_y; // assign the left drive power

        gp1_right_stick_y = Range.clip(gamepad1.right_stick_y, -1, 1); // update the right joystick value
        right_drive_power = gp1_right_stick_y; // set the right drive power

        if(!autoOn) {
            r.setPower(right_drive_power);
            rb.setPower(right_drive_power);
            l.setPower(left_drive_power);
            lb.setPower(left_drive_power);
        }

        //Lift Control
        if (!autoOn && gamepad1.dpad_up) {
            lift.setPower(1);
        } // else if is important - it prevents the system from freaking out
        else if (!autoOn && gamepad1.dpad_down) {
            lift.setPower(-1);
        } else {lift.setPower(0);}
        // Emergency Brake for the lift
        if (!autoOn && gamepad1.y) {
            lift.setPower(0);
        }

        // Beacon Pusher Control
        if (!autoOn && gamepad1.left_bumper) {
            button_left.setPosition(0.05);
        } // notice how there isn't an else if here - both actions can happen at once
        if (!autoOn && gamepad1.right_bumper) {
            button_right.setPosition(0.95);
        }
        // Reset to original position
        if (!autoOn && gamepad1.x) {
            button_left.setPosition(0.9);
            button_right.setPosition(0.1);
        }

        // Flywheel control
        if (!autoOn && gamepad1.b) {
            fly.setPower(0); // Emergency stop the flywheel
        }
        else if (!autoOn && gamepad1.left_trigger > 0) {
            fly.setPower(Range.clip(gamepad1.left_trigger*2, 0, 0.5)); /// change the last number to increase max power
        } else {fly.setPower(0);}
    }

    @Override
    public void stop() {
        l.setPower(0);
        lb.setPower(0);
        r.setPower(0);
        rb.setPower(0);
        lift.setPower(0);
        fly.setPower(0);
        super.stop();
    }
}
