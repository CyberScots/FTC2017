/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left claw" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "JackDrive", group = "Cyber Scots")
//@Disabled
public class JackDrive extends LinearOpMode {
    static final double INCREMENT   = 0.01;     // amount to slow servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   25;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    double CLAW_POS = 0;
    ColorSensor colorSensor;
    // Define class members
    public DcMotor leftDrive   = null;
    public DcMotor rightDrive   = null;
    public DcMotor arm   = null;
    public Servo leftHand = null;
    public Servo rightHand = null;
    double  motorPowerL = 0;
    double motorPowerR = 0;


    //@Override
    public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive  = hardwareMap.get(DcMotor.class, "right_drive");
        arm  = hardwareMap.get(DcMotor.class, "left_arm");
        leftHand  = hardwareMap.get(Servo.class, "left_hand");
        rightHand  = hardwareMap.get(Servo.class, "right_hand");
        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        arm.setDirection(DcMotor.Direction.FORWARD);
        leftHand.setPosition(0.5);
        rightHand.setPosition(0.5);
        // Wait for the start button
        telemetry.addData(">", "Press Start to move Servo." );
        telemetry.update();
        waitForStart();


        while(opModeIsActive()){

            motorPowerL = gamepad1.left_trigger; // original code
            motorPowerR = gamepad1.right_trigger;

            if (Math.abs(motorPowerL) < 0.05) {
                motorPowerL = 0;
            }
            if (Math.abs(motorPowerR) < 0.05) {
                motorPowerR = 0;
            }
            if (gamepad1.y) {
                arm.setPower(0.5);
            }else if (gamepad1.a) {
                arm.setPower(-0.5);
            }else {
                arm.setPower(0);
            }

            CLAW_POS += (gamepad1.right_trigger - gamepad1.left_trigger)/4;
            CLAW_POS = Range.clip(CLAW_POS, 0, 0.5);
            leftHand.setPosition(0.5 + CLAW_POS);
            rightHand.setPosition(0.5 - CLAW_POS);

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
}
