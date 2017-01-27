package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;


/**

 * Created by alexbulanov on 9/28/16.

 */

@TeleOp(name = "DriveAuto")
public class DriveAutoBeacon extends LinearOpMode {
    DcMotor l;
    DcMotor r;
    DcMotor lb;
    DcMotor rb;
    DcMotor lift;
    DcMotor fly;
    OpticalDistanceSensor eodsBack;
    OpticalDistanceSensor eodsFore;
    ColorSensor color_left;
    Servo button_left;
    Servo button_right;
    Servo wall_servo;
    Servo fly_servo;
    TouchSensor touch;
    boolean autoOn = false;
    double powerScale = 1;

    public DriveAutoBeacon() {}

    double scale_motor_power(double p_power)  //Scales joystick value to output appropriate motor power
    {                                          //Use like "scale_motor_power(gamepad1.left_stick_x)"
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
        return l_scale * powerScale;
    }

    @Override
    public void runOpMode() throws InterruptedException{
        l = hardwareMap.dcMotor.get("l");
        r = hardwareMap.dcMotor.get("r");
        rb = hardwareMap.dcMotor.get("rb");
        lb = hardwareMap.dcMotor.get("lb");
        lift = hardwareMap.dcMotor.get("lift");
        fly = hardwareMap.dcMotor.get("fly");

        button_left = hardwareMap.servo.get("bl");
        button_right = hardwareMap.servo.get("br");
        eodsFore = hardwareMap.opticalDistanceSensor.get("eodsF");
        eodsBack = hardwareMap.opticalDistanceSensor.get("eodsB");
        color_left = hardwareMap.colorSensor.get("cl");
        l.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        touch = hardwareMap.touchSensor.get("t");
        wall_servo = hardwareMap.servo.get("ws");
        fly_servo = hardwareMap.servo.get("sf");

        waitForStart();
        while (opModeIsActive()) {

            float l_gp1_left_stick_y = gamepad1.left_stick_y;
            float l_left_drive_power
                    = (float) scale_motor_power(l_gp1_left_stick_y);

            float l_gp1_right_stick_y = gamepad1.right_stick_y;
            float l_right_drive_power
                    = (float) scale_motor_power(l_gp1_right_stick_y);
            if(!autoOn) {
                r.setPower(l_right_drive_power);
                l.setPower(l_left_drive_power);
                rb.setPower(l_right_drive_power);
                lb.setPower(l_left_drive_power);
            }
            if (!autoOn && gamepad1.y) {
                lift.setPower(0);
            }
            else if (!autoOn && gamepad1.dpad_up) {
                lift.setPower(1);
            }
            else if (!autoOn && gamepad1.dpad_down) {
                lift.setPower(-1);
            }
            if (!autoOn && gamepad1.left_bumper) {
                button_left.setPosition(0.01);
            }
            if (!autoOn && gamepad1.right_bumper) {
                button_right.setPosition(0.95 );
            }
            if (!autoOn && gamepad1.x) {
                button_left.setPosition(0.87);
                button_right.setPosition(0.1);
            }
            if (!autoOn && gamepad1.left_stick_button) {
                powerScale = 0.20d;
            }
            else if (!autoOn && gamepad1.right_stick_button) {
                powerScale = 1.0d;
            }
            if (!autoOn && gamepad1.a) {
                fly.setPower(-0.95);
            }
            else if (!autoOn && gamepad1.b) {
                fly.setPower(0);
            }

            if(!autoOn && gamepad1.dpad_left) {
                fly_servo.setPosition(0.05);
            }
            else if (!autoOn && gamepad1.dpad_right) {
                fly_servo.setPosition(0.95);
            }

            if(!autoOn && gamepad1.right_trigger >= 0.5) {
                pressBeaconSided("right");
            }
            else if(!autoOn && gamepad1.left_trigger >= 0.5) {
                pressBeaconSided("left");
            }
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
        touch.close();
        eodsFore.close();
        eodsBack.close();
        color_left.close();
        lift.close();
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

    public boolean pressBeaconSided(String side) throws InterruptedException{
        //Sets Initial Servo Positions
        autoOn = true;
        wall_servo.setPosition(0.39);
        button_right.setPosition(0.1);
        button_left.setPosition(0.9);
        color_left.enableLed(false);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Moves to Line from Start
        while (eodsFore.getLightDetected() < 0.03 && continuePressSided(side)) { drive(0.2); }
        if (!opModeIsActive()) return returnBeaconPress();
        while(eodsBack.getLightDetected() < 0.03 && continuePressSided(side)) { drive(0.12); }
        if (!opModeIsActive()) return returnBeaconPress();
        while(eodsBack.getLightDetected() > 0.03 && continuePressSided(side)) { drive(0.12); }
        stopDrive();
        while (eodsFore.getLightDetected() < 0.03 && continuePressSided(side)) {
            if (side.equalsIgnoreCase("left")) {
                setLeftPower(-0.15);
                setRightPower(0.15);
            }
            else if (side.equalsIgnoreCase("right")) {
                setLeftPower(0.15);
                setRightPower(-0.15);
            }
        }
        if (!continuePressSided(side)) return returnBeaconPress();
        stopDrive();
        //Move to wall
        while (!touch.isPressed() && continuePressSided(side)) { drive(0.2); }
        stopDrive();
        if (!continuePressSided(side)) return returnBeaconPress();
        //Gets First's Beacon color, true if red, false if blue
        boolean colorLeftSide = color_left.red() > color_left.blue();
        //Drives back from Beacon
        drive(-0.12);
        sleepOpModeSided(600, side);
        if (!continuePressSided(side)) return returnBeaconPress();
        stopDrive();
        //Retracts button
        wall_servo.setPosition(0.1);
        if (!continuePressSided(side)) return returnBeaconPress();
        //Deploys pusher servos
        if (colorLeftSide) { button_right.setPosition(0.95);
        } else { button_left.setPosition(0.01); }
        //Waits for servos to move
        sleepOpModeSided(350, side);
        stopDrive();
        if (!continuePressSided(side)) return returnBeaconPress();
        //Drives forward and presses button
        drive(0.13);
        sleepOpModeSided(1525, side);
        if (!continuePressSided(side)) return returnBeaconPress();
        stopDrive();
        //Drives Back
        drive(-0.2);
        sleepOpModeSided(400, side);
        stopDrive();
        if (!continuePressSided(side)) return returnBeaconPress();
        //Sets Servos
        wall_servo.setPosition(0.37);
        button_right.setPosition(0.1);
        button_left.setPosition(0.9);
        sleepOpModeSided(350, side);
        return returnBeaconPress();
    }

    public boolean returnBeaconPress() {
        autoOn = false;
        return true;
    }

    public boolean continuePressSided(String side) {
        return opModeIsActive() && (side.equalsIgnoreCase("right") ? gamepad1.right_bumper : gamepad1.left_bumper);
    }

    public void sleepOpModeSided(double millTime, String side) throws InterruptedException {
        double time = System.currentTimeMillis();
        while (continuePressSided(side) && System.currentTimeMillis() < time + millTime) {
            this.sleep(1);
        }
    }
}