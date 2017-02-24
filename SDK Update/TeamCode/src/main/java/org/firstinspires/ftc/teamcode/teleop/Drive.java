package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by alexbulanov on 9/28/16.
 */

@TeleOp(name = "Drive")
@Disabled
public class Drive extends LinearOpMode {
    DcMotor l;
    DcMotor r;
    DcMotor lb;
    DcMotor rb;
    DcMotor lift;
    Servo button_left;
    Servo button_right;

    public Drive() {}

    double scale_motor_power(double p_power) {
        //Scales joystick value to output appropriate motor power
        // Use like "scale_motor_power(gamepad1.left_stick_x)"
        //
        // Assume no scaling
        //
        double l_scale = 0.0;
        //
        // Ensure the values are legal
        //
        double l_power = Range.clip(p_power, -1, 1);
        double[] l_array =
                {0.00, 0.05, 0.09, 0.10, 0.12
                        , 0.15, 0.18, 0.24, 0.30, 0.36
                        , 0.43, 0.50, 0.60, 0.72, 0.85
                        , 1.00, 1.00
                };
        //
        // Get the corresponding index for the specified argument/parameter.
        //
        int l_index = (int) (l_power * 16.0);

        if (l_index < 0) {
            l_index = -l_index;
        } else if (l_index > 16) {
            l_index = 16;
       }

        if (l_power < 0) {
            l_scale = -l_array[l_index];
        } else {
            l_scale = l_array[l_index];
        }
        return l_scale;
    }

    @Override
    public void runOpMode() {

        while (true) {
            l = hardwareMap.dcMotor.get("l");
            r = hardwareMap.dcMotor.get("r");
            rb = hardwareMap.dcMotor.get("rb");
            lb = hardwareMap.dcMotor.get("lb");
            lift = hardwareMap.dcMotor.get("lift");
            button_left = hardwareMap.servo.get("bl");
            button_right = hardwareMap.servo.get("br");

            r.setDirection(DcMotor.Direction.REVERSE);
            rb.setDirection(DcMotor.Direction.REVERSE);

            float l_gp1_left_stick_y = gamepad1.left_stick_y;
            float l_left_drive_power
                    = (float) scale_motor_power(l_gp1_left_stick_y);

            float l_gp1_right_stick_y = gamepad1.right_stick_y;
            float l_right_drive_power
                    = (float) scale_motor_power(l_gp1_right_stick_y);

            r.setPower(l_right_drive_power);
            l.setPower(l_left_drive_power);
            rb.setPower(l_right_drive_power);
            lb.setPower(l_left_drive_power);

            if (gamepad1.dpad_up) {
                lift.setPower(1);
            }
            if (gamepad1.dpad_down) {
                lift.setPower(-1);
            }
            if (gamepad1.y) {
                lift.setPower(0);
            }
            if (gamepad1.left_bumper) {
                button_left.setPosition(0.05);
            }
            if (gamepad1.right_bumper) {
                button_right.setPosition(0.95 );
            }
            if (gamepad1.b) {
                button_left.setPosition(0.9);
                button_right.setPosition(0.1);
            }

        }
    }
}