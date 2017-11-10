package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name = "JackDrive", group = "Cyber Scots")
//@Disabled
public class JackDrive extends LinearOpMode {
    static final double INCREMENT   = 0.01;     // amount to slow servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   25;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    public static final boolean mode1 = false;
    public static final boolean mode2 = true;
    double CLAW_POS = 0;
    TouchSensor lefttouch;
    TouchSensor righttouch;
    ColorSensor colorSensor;
    // Define class members
    public DcMotor leftDrive   = null;
    public DcMotor rightDrive   = null;
    public DcMotor arm   = null;
    public Servo leftHand = null;
    public Servo rightHand = null;
    double  motorPowerL = 0;
    double motorPowerR = 0;
    double motorarm = 0;


    //@Override
    public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive  = hardwareMap.get(DcMotor.class, "right_drive");
        arm  = hardwareMap.get(DcMotor.class, "left_arm");
        leftHand  = hardwareMap.get(Servo.class, "left_hand");
        rightHand  = hardwareMap.get(Servo.class, "right_hand");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        leftHand.setPosition(0.5);
        rightHand.setPosition(0.5);
        boolean mode = mode1;
        // Wait for the start button
        telemetry.addData(">", "Press Start to move Servo." );
        telemetry.update();

        waitForStart();


        while(opModeIsActive()){
            lefttouch = hardwareMap.get(TouchSensor.class, "Sensor_left");
            righttouch = hardwareMap.get(TouchSensor.class, "Sensor_right");
            motorPowerL = gamepad1.left_trigger;
            motorPowerR = gamepad1.right_trigger;
            motorarm = gamepad1.right_stick_y;
            if (gamepad1.x) {mode = mode1;}
            if (gamepad1.b) {mode = mode2;}
            if (mode==mode1) {if (Math.abs(motorPowerL) < 0.05) {
                motorPowerL = 0;
            }
                if (Math.abs(motorPowerR) < 0.05) {
                    motorPowerR = 0;
                }
                if (gamepad1.right_stick_y > 0.05) {
                    arm.setPower(0);
                }else {
                    arm.setPower(motorarm);
                }

                if (!lefttouch.isPressed()&& !righttouch.isPressed()) {
                    CLAW_POS += (gamepad1.left_stick_x)/4;
                    CLAW_POS = Range.clip(CLAW_POS, 0, 0.5);
                    leftHand.setPosition(0 + CLAW_POS);
                    rightHand.setPosition(0 - CLAW_POS);
                };

                // Display the current value
                telemetry.addData(">", "Press Stop to end test." );
                telemetry.update();

                // Set the servo to the new position and pause;

                leftDrive.setPower(motorPowerL);
                rightDrive.setPower(motorPowerR);
                sleep(CYCLE_MS);
                idle();
            }

            // Signal done;
            telemetry.addData(">", "Done");
            telemetry.update();
        }
    }};

