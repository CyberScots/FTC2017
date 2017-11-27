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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;

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
@TeleOp(name = "RagBot MecanumDrive", group = "Cyber Scots")
//@Disabled
public class ZorbDriveMecanum extends LinearOpMode {
    static final double INCREMENT   = 0.01;     // amount to slow servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   25;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    double CLAW_POS = 0;
    // Define class members
    public DcMotor leftFront   = null;
    public DcMotor rightFront   = null;
    public DcMotor leftBack   = null;
    public DcMotor rightBack   = null;
    public DcMotor lBelt   = null;
    public DcMotor rBelt   = null;
    public Servo leftClose   = null;
    public Servo rightClose   = null;
    double  motorFL = 0;
    double motorFR = 0;
    double  motorBL = 0;
    double motorBR = 0;
    double joystickForward = 0;
    double joystickTurn = 0;
    double joystickSide = 0;
    static final double     FORWARD_SPEED = 1;
    static final double     TURN_SPEED    = 0.5;
    private ElapsedTime runtime = new ElapsedTime();

    //@Override
    public void runOpMode() {

        /*

          _____                    _____                    _____                    _____                   _______               _____
         /\    \                  /\    \                  /\    \                  /\    \                 /::\    \             /\    \
        /::\    \                /::\    \                /::\    \                /::\    \               /::::\    \           /::\    \
       /::::\    \              /::::\    \              /::::\    \              /::::\    \             /::::::\    \          \:::\    \
      /::::::\    \            /::::::\    \            /::::::\    \            /::::::\    \           /::::::::\    \          \:::\    \
     /:::/\:::\    \          /:::/\:::\    \          /:::/\:::\    \          /:::/\:::\    \         /:::/~~\:::\    \          \:::\    \
    /:::/__\:::\    \        /:::/__\:::\    \        /:::/  \:::\    \        /:::/__\:::\    \       /:::/    \:::\    \          \:::\    \
   /::::\   \:::\    \      /::::\   \:::\    \      /:::/    \:::\    \      /::::\   \:::\    \     /:::/    / \:::\    \         /::::\    \
  /::::::\   \:::\    \    /::::::\   \:::\    \    /:::/    / \:::\    \    /::::::\   \:::\    \   /:::/____/   \:::\____\       /::::::\    \
 /:::/\:::\   \:::\____\  /:::/\:::\   \:::\    \  /:::/    /   \:::\ ___\  /:::/\:::\   \:::\ ___\ |:::|    |     |:::|    |     /:::/\:::\    \
/:::/  \:::\   \:::|    |/:::/  \:::\   \:::\____\/:::/____/  ___\:::|    |/:::/__\:::\   \:::|    ||:::|____|     |:::|    |    /:::/  \:::\____\
\::/   |::::\  /:::|____|\::/    \:::\  /:::/    /\:::\    \ /\  /:::|____|\:::\   \:::\  /:::|____| \:::\    \   /:::/    /    /:::/    \::/    /
 \/____|:::::\/:::/    /  \/____/ \:::\/:::/    /  \:::\    /::\ \::/    /  \:::\   \:::\/:::/    /   \:::\    \ /:::/    /    /:::/    / \/____/
       |:::::::::/    /            \::::::/    /    \:::\   \:::\ \/____/    \:::\   \::::::/    /     \:::\    /:::/    /    /:::/    /
       |::|\::::/    /              \::::/    /      \:::\   \:::\____\       \:::\   \::::/    /       \:::\__/:::/    /    /:::/    /
       |::| \::/____/               /:::/    /        \:::\  /:::/    /        \:::\  /:::/    /         \::::::::/    /     \::/    /
       |::|  ~|                    /:::/    /          \:::\/:::/    /          \:::\/:::/    /           \::::::/    /       \/____/
       |::|   |                   /:::/    /            \::::::/    /            \::::::/    /             \::::/    /
       \::|   |                  /:::/    /              \::::/    /              \::::/    /               \::/____/
        \:|   |                  \::/    /                \::/____/                \::/____/                 ~~
         \|___|                   \/____/                                           ~~


        */

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        rightFront  = hardwareMap.get(DcMotor.class, "right_front");
        leftBack  = hardwareMap.get(DcMotor.class, "left_back");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back");
        lBelt  = hardwareMap.get(DcMotor.class, "left_belt");
        rBelt  = hardwareMap.get(DcMotor.class, "right_belt");
        rightClose  = hardwareMap.get(Servo.class, "right_close");
        leftClose  = hardwareMap.get(Servo.class, "left_close");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        lBelt.setDirection(DcMotor.Direction.FORWARD);
        lBelt.setPower(0);
        rBelt.setDirection(DcMotor.Direction.REVERSE);
        rBelt.setPower(0);
        leftClose.setPosition(0.5);
        rightClose.setPosition(0.5);
        // Wait for the start button0

        telemetry.addData(">", "Press Start to use Zorb's awesome drive for the Ragbot" );
        telemetry.addData(">", "    ____               __          __ " );
        telemetry.addData(">", "   / __ \\____ _____ _/ /_  ____  / /_" );
        telemetry.addData(">", "  / /_/ / __ `/ __ `/ __ \\/ __ \\/ __/" );
        telemetry.addData(">", " / _, _/ /_/ / /_/ / /_/ / /_/ / /_  " );
        telemetry.addData(">", "/_/ |_|\\__,_/\\__, /_.___/\\____/\\__/  " );
        telemetry.addData(">", "            /____/                   " );
        telemetry.update();
        waitForStart();


        while(opModeIsActive()){
            joystickForward = gamepad1.right_stick_y;
            joystickSide = gamepad1.right_stick_x;
            joystickTurn = gamepad1.left_stick_x;

            if (Math.abs(joystickForward) < 0.01) {
                joystickForward = 0;
            }
            if (Math.abs(joystickTurn) < 0.01) {
                joystickTurn = 0;
            }
            motorFL = Math.pow(joystickForward, 5);
            motorFR =  Math.pow(joystickForward, 5);
            motorBL = Math.pow(joystickForward, 5);
            motorBR =  Math.pow(joystickForward, 5);

            if (joystickSide > 0 && joystickForward > 0) {
                motorFR -= joystickSide;
                motorBL -= joystickSide;
            }
            if (joystickSide > 0 && joystickForward < 0) {
                motorFR += joystickSide;
                motorBL += joystickSide;
            }
            if (joystickSide < 0 && joystickForward > 0) {
                motorFL -= joystickSide;
                motorBR -= joystickSide;
            }
            if (joystickSide < 0 && joystickForward < 0) {
                motorFL += joystickSide;
                motorBR += joystickSide;
            }

            motorFL -= Math.pow(joystickTurn, 5)*TURN_SPEED;
            motorFR += Math.pow(joystickTurn, 5)*TURN_SPEED;
            motorBL -= Math.pow(joystickTurn, 5)*TURN_SPEED;
            motorBR += Math.pow(joystickTurn, 5)*TURN_SPEED;

            motorFL = Range.clip(motorFL, -1, 1);
            motorFR = Range.clip(motorFR, -1, 1);
            motorBL = Range.clip(motorBL, -1, 1);
            motorBR = Range.clip(motorBR, -1, 1);



            lBelt.setPower(-Math.pow(gamepad1.left_stick_y, 5));
            rBelt.setPower(-Math.pow(gamepad1.left_stick_y, 5));


            CLAW_POS += Math.pow((gamepad1.right_trigger - gamepad1.left_trigger),5);
            CLAW_POS = Range.clip(CLAW_POS, 0, 0.5);
            leftClose.setPosition(0.5 + CLAW_POS);
            rightClose.setPosition(0.5 - CLAW_POS);

            // Display the current value
            telemetry.addData(">", "Press Stop to end Zorb's epic drive." );
            telemetry.update();

            // Set the servo to the new position and pause;

            leftFront.setPower(motorFL);
            rightFront.setPower(motorFR);
            leftBack.setPower(motorBL);
            rightBack.setPower(motorBR);
/*
            if (gamepad1.x) {
                arm.setPower(0);
                //drive forward
                leftDrive.setPower(FORWARD_SPEED);
                rightDrive.setPower(FORWARD_SPEED);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < .5)) {
                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                    telemetry.update();
                }

                // Step 2:  Spin right for 2 seconds
                leftDrive.setPower(-TURN_SPEED);
                rightDrive.setPower(TURN_SPEED);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 2.2)) {
                    telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
                    telemetry.update();
                }

                // Step 3:  Drive Back for .5 seconds
                leftDrive.setPower(-FORWARD_SPEED);
                rightDrive.setPower(-FORWARD_SPEED);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < .5)) {
                    telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                //lower the arm
                // Step 4:  Stop
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                telemetry.addData("Path", "Complete");
                telemetry.update();
            }


*/
            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}