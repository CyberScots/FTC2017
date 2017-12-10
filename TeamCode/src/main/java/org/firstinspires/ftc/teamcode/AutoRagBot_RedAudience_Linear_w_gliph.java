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
//test
package org.firstinspires.ftc.teamcode;

 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.Disabled;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.util.ElapsedTime;

 import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

 /**
  * This file illustrates the concept of driving a path based on time.
  * It uses the common Pushbot hardware class to define the drive on the robot.
  * The code is structured as a LinearOpMode
  *
  * The code assumes that you do NOT have encoders on the wheels,
  *   otherwise you would use: PushbotAutoDriveByEncoder;
  *
  *   The desired path in this example is:
  *   - Drive forward for 3 seconds
  *   - Spin right for 1.3 seconds
  *   - Drive Backwards for 1 Second
  *   - Stop and close the claw.
  *
  *  The code is written in a simple form with no optimizations.
  *  However, there are several ways that this type of sequence could be streamlined,
  *
  * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
  * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
  */
//setup the code
 @Autonomous(name="Glyph Auto Rag Bot Red;+:Audience", group="Pushbot")
 @Disabled
 public class AutoRagBot_RedAudience_Linear_w_gliph extends LinearOpMode {

     /* Declare OpMode members. */
     HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
     private ElapsedTime     runtime = new ElapsedTime();


     static final double     FORWARD_SPEED = 0.45;
     static final double     TURN_SPEED    = -.18;

     //@Override
     public void runOpMode() {
         //Setup robot
         /*
          * Initialize the drive system variables.
          * The init() method of the hardware class does all the work here
          */
         robot.init(hardwareMap);
         robot.leftClaw.setPosition(.7);
         robot.rightClaw.setPosition(-.7);
         // Send telemetry message to signify robot waiting;
         telemetry.addData("Status", "Hacking your phone");    //
         telemetry.update();

         // Wait for the game to start (driver presses PLAY)
         waitForStart();
        // robot.leftClaw.setPosition(.7);
       //  robot.rightClaw.setPosition(-.7);
         //point the arm up to make moving easy
         runtime.reset();
        // robot.leftArm.setPower(0.3);
         while (opModeIsActive() && (runtime.seconds() < 2)) {
             telemetry.addData("Path", "Moving arm", runtime.seconds());
             telemetry.update();
         }
         robot.leftArm.setPower(0);
         // Step 1: Drive forward
         robot.leftDrive.setPower(FORWARD_SPEED);
         robot.rightDrive.setPower(FORWARD_SPEED);
         runtime.reset();
         while (opModeIsActive() && (runtime.seconds() < 2.625)) {
             telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
             telemetry.update();
         }
         // Step 2:  Spin right for 2 seconds

       //  robot.leftDrive.setPower(-TURN_SPEED);
        // robot.rightDrive.setPower(TURN_SPEED);
        // runtime.reset();
        // while (opModeIsActive() && (runtime.seconds() < 2.2)) {
        //     telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
        //     telemetry.update();
       //  }
         // Step 3:  Drive forward for .5 seconds

         robot.leftDrive.setPower(FORWARD_SPEED);
         robot.rightDrive.setPower(FORWARD_SPEED);
         runtime.reset();
         while (opModeIsActive() && (runtime.seconds() < .24)) {
             telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
             telemetry.update();
         }
         robot.leftDrive.setPower(0);
         robot.rightDrive.setPower(0);

         robot.leftArm.setPower(0);
         // Step 4:  Stop
         robot.leftDrive.setPower(0);
         robot.rightDrive.setPower(0);
         runtime.reset();
         //robot.leftArm.setPower(0.3);
         while (opModeIsActive() && (runtime.seconds() < 2)) {
             telemetry.addData("Path", "Moving arm", runtime.seconds());
             telemetry.update();
         }
        // robot.leftClaw.setPosition(.5);
        // robot.rightClaw.setPosition(.5);
         telemetry.addData("Path", "Complete");
         telemetry.update();
         sleep(1000);
     }
 }
