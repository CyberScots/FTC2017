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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="Red Audience Ragbot", group ="Concept")
//@Disabled
public class AutoRagbotRedAudience extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";
    public DcMotor leftFront   = null;
    public DcMotor rightFront   = null;
    public DcMotor leftBack   = null;
    public DcMotor rightBack   = null;
    public DcMotor lBelt   = null;
    public DcMotor rBelt   = null;
    public Servo leftClose   = null;
    public Servo rightClose   = null;
    ColorSensor sensorColor;
    double  motorFL = 0;
    double motorFR = 0;
    double  motorBL = 0;
    double motorBR = 0;
    static final double     FORWARD_SPEED = 1;
    static final double     TURN_SPEED    = 0.5;
    private ElapsedTime runtime = new ElapsedTime();

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    public static double getAngle(double x, double y)
    {
        //return Math.atan2(y,x);
        //return ((1.5 * Math.PI - Math.atan2(y,x))/Math.PI)-1;
        return (1.5 * Math.PI - Math.atan2(-y,-x));
    }

    public void mecanum(double dir, double speed, double turn) {
        motorFL = speed*Math.sin(/*2*Math.PI**/dir + Math.PI/4) + turn;
        motorBR = speed*Math.sin(/*2*Math.PI**/dir + Math.PI/4) - turn;
        motorFR = speed*Math.cos(/*2*Math.PI**/dir + Math.PI/4) - turn;
        motorBL = speed*Math.cos(/*2*Math.PI**/dir + Math.PI/4) + turn;
        telemetry.addData("Direction:", dir);
    }

    public void move(double x, double y, double turn) {
        telemetry.addData("x axis movement", x);
        telemetry.addData("y axis movement", y);
        telemetry.addData("Turning", turn);
        turn -= x;
        mecanum(getAngle(x,y), Math.sqrt(Math.pow(x, 2) + Math.pow(y ,2)), turn);

        telemetry.addData("motorFR", motorFR);
        telemetry.addData("motorBL", motorBL);
        telemetry.addData("motorFL", motorFL);
        telemetry.addData("motorBR", motorBR);

        leftFront.setPower(motorFL);
        rightFront.setPower(motorFR);
        leftBack.setPower(motorBL);
        rightBack.setPower(motorBR);
    }

    @Override public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AcRNMjL/////AAAAGYx9swjI4EM3gOz2yIkuui5Eo0LMsIsAxmD+X+Lz2Eox41tmaut+zNhNGm68NGyXSnmYIwcWSIVz/fOZf+ht++8XJScyjQv/BDbKKOOEZ3//KzhxFkS93SlQ3OKX+KhDdtv/USecJsSYAMY/A77pOHu10H6SXHGC2fTuCa1+mzp6rEpugFFC0JxcTJSyFTx3IMvH4BPU98zZbTbb8bnVl1usz84xusFUTKGua19+lvZ1gBwfe/SltwgQZEmzTrQPT7K8cnu0obpmxspet8k5FHbqeJvzXV9PMK1wd2+wYygYQMeJCOrg/ZIE/fRODW4sgDIt6L85XMehoidJ3aE2csreAsiSaQsFgnYe4H07XwDi";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
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


        telemetry.addData(">", "Press Start to use Zorb's awesome VuForia for the Ragbot" );
        telemetry.addData(">", "    ____               __          __ " );
        telemetry.addData(">", "   / __ \\____ _____ _/ /_  ____  / /_" );
        telemetry.addData(">", "  / /_/ / __ `/ __ `/ __ \\/ __ \\/ __/" );
        telemetry.addData(">", " / _, _/ /_/ / /_/ / /_/ / /_/ / /_  " );
        telemetry.addData(">", "/_/ |_|\\__,_/\\__, /_.___/\\____/\\__/  " );
        telemetry.addData(">", "            /____/                   " );
        telemetry.update();
        waitForStart();

        runtime.reset();

        relicTrackables.activate();

/*
        if (sensorColor.red() > sensorColor.blue()) {
        } else if (sensorColor.red() < sensorColor.blue()) {
        }*/

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.4)) {
            move(-1,0,0);
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            move(0,1,0);
        }


        leftClose.setPosition(0.2);
        rightClose.setPosition(0.8);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            move(0,-0.5,0);
        }
move(0,0,0);
/*
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < 2)) {
            move(0.5,1.5,0);
        }
        move(0,0,0);*/
            /*RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    telemetry.addData("VuMark Detected: ", "Left");
                    move(-1,0,0);
                }
                if (vuMark == RelicRecoveryVuMark.CENTER) {
                    telemetry.addData("VuMark Detected: ", "Center");
                    move(0,1,0);
                }
                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    telemetry.addData("VuMark Detected: ", "Right");
                    move(1,0,0);
                }
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "Not found ):");
                move(0,0,0);
            }*/


            /*telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());*/


            telemetry.update();
        }


    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
