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

@TeleOp(name = "TeleOpMain")
public class TeleOpMain extends LinearOpMode {

    DcMotor l;
    DcMotor r;
    DcMotor lb;
    DcMotor rb;
    DcMotor lift;
    DcMotor fly;
    DcMotor loader;
    DcMotor intake;
    OpticalDistanceSensor eodsBack;
    OpticalDistanceSensor eodsFore;
    ColorSensor color_left;
    Servo button_left;
    Servo button_right;
    Servo fly_servo;
    TouchSensor touch;
    double powerScale = 1;
    boolean firing = false;
    boolean loading = false;
    boolean loading2 = false;
    boolean togglePower = false;
    boolean lBPressed = false;
    boolean rBPressed = false;
    boolean lBOut = false;
    boolean rBOut = false;
    long tPStart;
    long firingStart;
    long lBStart;
    long rBStart;
    long uLiftStart;
    long dLiftStart;
    double liftPower;

    public TeleOpMain() {}

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
        intake = hardwareMap.dcMotor.get("in");
        loader = hardwareMap.dcMotor.get("load");

        button_left = hardwareMap.servo.get("bl");
        button_right = hardwareMap.servo.get("br");
        eodsFore = hardwareMap.opticalDistanceSensor.get("eodsF");
        eodsBack = hardwareMap.opticalDistanceSensor.get("eodsB");
        color_left = hardwareMap.colorSensor.get("cl");
        r.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
        touch = hardwareMap.touchSensor.get("t");
        fly_servo = hardwareMap.servo.get("sf");
        tPStart = System.currentTimeMillis();
        firingStart = System.currentTimeMillis();
        rBStart = lBStart = uLiftStart = dLiftStart = System.currentTimeMillis();
        fly_servo.setPosition(0);
        liftPower = 0;
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1 == null) break;
            float l_gp1_left_stick_y = gamepad1.left_stick_y;
            float l_left_drive_power
                    = (float) -scale_motor_power(l_gp1_left_stick_y);

            float l_gp1_right_stick_y = gamepad1.right_stick_y;
            float l_right_drive_power
                    = (float) -scale_motor_power(l_gp1_right_stick_y);
            r.setPower(l_right_drive_power);
            l.setPower(l_left_drive_power);
            rb.setPower(l_right_drive_power);
            lb.setPower(l_left_drive_power);
            if (gamepad1.dpad_up) {
                lift.setPower(0.95);
            }
            else if(gamepad1.dpad_down) {
                lift.setPower(-0.95);
            }
            else {
                lift.setPower(0);
            }

            if (gamepad1.left_bumper && !lBPressed) {
                if (lBOut) button_left.setPosition(0.90);
                else button_left.setPosition(0.01);
                lBOut = !lBOut;
                lBStart = System.currentTimeMillis();
                lBPressed = true;
            } else if(lBPressed && System.currentTimeMillis() - lBStart > 320) {
                lBPressed = false;
            }

            if (gamepad1.right_bumper && !rBPressed) {
                if (rBOut) button_right.setPosition(0.92);
                else button_right.setPosition(0.15);
                rBOut = !rBOut;
                rBStart = System.currentTimeMillis();
                rBPressed = true;
            } else if(rBPressed && System.currentTimeMillis() - rBStart > 320) {
                rBPressed = false;
            }

            if (gamepad1.left_stick_button && !togglePower) {
                powerScale = powerScale == 1f ? 0.2f : 1f;
                togglePower = true;
                tPStart = System.currentTimeMillis();
            }
            else if(togglePower && System.currentTimeMillis() - tPStart > 350) {
                togglePower = false;
            }

            if(gamepad1.left_trigger > 0.5) intake.setPower(-0.9);
            else intake.setPower(0);

            if(gamepad1.right_trigger > 0.5 && !firing) {
                firing = true;
                loading = true;
                loading2 = false;
                firingStart = System.currentTimeMillis();
                turnLoader(0.6f, 4);
                fly.setPower(-0.95);
            }
            if(firing && loading && !loading2) {
                if (!loader.isBusy()) loading = false;
            }
            if(firing && !loading && !(System.currentTimeMillis() - firingStart > 6500) && !loading2) {
                fly_servo.setPosition(0.95);
            }
            else if (firing && !loading &&!loading2 && System.currentTimeMillis() - firingStart > 6500) {
                fly_servo.setPosition(0.05);
                fly.setPower(0);
                turnLoader(-0.6f, -4);
                loading2 = true;
                loading = false;
            }
            else if(firing && loading2) {
                if(!loader.isBusy()) {
                    firing = false;
                    loading2 = false;
                    loader.setPower(0);
                }
            }

        }
        ceaseDrive();
        l.close();
        r.close();
        lb.close();
        rb.close();
        fly.close();
        intake.close();
        loader.close();
        button_left.close();
        button_right.close();
        fly_servo.close();
        touch.close();
        eodsFore.close();
        eodsBack.close();
        color_left.close();
        lift.close();
        super.stop();
    }
    public void drive(double power) {
        l.setPower(power);
        r.setPower(power);
        lb.setPower(power);
        rb.setPower(power);
    }

    public void ceaseDrive() {
        drive(0);
        lift.setPower(0);
        intake.setPower(0);
        loader.setPower(0);
        fly.setPower(0);
    }

    public void turnLoader(float power, double rotations) {
        loader.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        loader.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        loader.setPower(power);
        loader.setTargetPosition((int) ((rotations*1440)+loader.getCurrentPosition()));
    }
}