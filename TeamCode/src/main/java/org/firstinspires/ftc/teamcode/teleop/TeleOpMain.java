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
    DcMotor intake;
    OpticalDistanceSensor eodsBack;
    OpticalDistanceSensor eodsFore;
    ColorSensor color_left;
    Servo button_left;
    Servo button_right;
    Servo fly_servo;
    Servo wall_servo;
    TouchSensor touch;
    double powerScale = 1;
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

        if (l_index < 0) l_index = -l_index;
        else if (l_index > 16) l_index = 16;

        l_scale = l_power < 0 ? -l_array[l_index] : l_array[l_index];

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

        button_left = hardwareMap.servo.get("bl");
        button_right = hardwareMap.servo.get("br");
        eodsFore = hardwareMap.opticalDistanceSensor.get("eodsF");
        eodsBack = hardwareMap.opticalDistanceSensor.get("eodsB");
        color_left = hardwareMap.colorSensor.get("cl");
        r.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
        touch = hardwareMap.touchSensor.get("t");
        fly_servo = hardwareMap.servo.get("sf");
        wall_servo = hardwareMap.servo.get("ws");
        tPStart = firingStart = rBStart = lBStart = uLiftStart = dLiftStart = System.currentTimeMillis();
        wall_servo.setPosition(0.2);
        liftPower = 0;
        waitForStart();
        while (opModeIsActive()) {
            //Null Check
            if (gamepad1 == null) break;

            //Sets Motor Powers
            setRightPower(-scale_motor_power(gamepad1.right_stick_y));
            setLeftPower(-scale_motor_power(gamepad1.left_stick_y));

            //Hold-press for lift
            if (gamepad1.dpad_up) lift.setPower(0.95);
            else if(gamepad1.dpad_down) lift.setPower(-0.95);
            else lift.setPower(0);

            //Toggle press for left pusher
            if (gamepad1.left_bumper && !lBPressed) {
                if (lBOut) button_left.setPosition(0.90);
                else button_left.setPosition(0.01);
                lBOut = !lBOut;
                lBStart = System.currentTimeMillis();
                lBPressed = true;
            } else if(lBPressed && System.currentTimeMillis() - lBStart > 320) lBPressed = false;

            //Toggle press for right pusher
            if (gamepad1.right_bumper && !rBPressed) {
                if (rBOut) button_right.setPosition(0.92);
                else button_right.setPosition(0.15);
                rBOut = !rBOut;
                rBStart = System.currentTimeMillis();
                rBPressed = true;
            } else if(rBPressed && System.currentTimeMillis() - rBStart > 320) rBPressed = false;

            //Toggle press for power scale, left stick
            if (gamepad1.left_stick_button && !togglePower) {
                powerScale = powerScale == 1f ? 0.2f : 1f;
                togglePower = true;
                tPStart = System.currentTimeMillis();
            } else if(togglePower && System.currentTimeMillis() - tPStart > 350)  togglePower = false;

            //Hold press for intake
            if(gamepad1.left_trigger > 0.5) intake.setPower(-0.95);
            else intake.setPower(0);

            //Hold-Press for fly-wheel
            if(gamepad1.right_trigger > 0.5) fly.setPower(-0.95);
            else fly.setPower(0);

            //Experimental Controls for Wall Servo Debug
            if(gamepad1.a) wall_servo.setPosition(0.8);
            else if (gamepad1.b)  wall_servo.setPosition(0.2);

        }
        //Stops Robot
        ceaseDrive();
        l.close();
        r.close();
        lb.close();
        rb.close();
        fly.close();
        intake.close();
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

    public void setLeftPower(double power) {
        l.setPower(power);
        lb.setPower(power);
    }

    public void setRightPower(double power) {
        r.setPower(power);
        rb.setPower(power);
    }

    public void ceaseDrive() {
        drive(0);
        lift.setPower(0);
        intake.setPower(0);
        fly.setPower(0);
    }
}