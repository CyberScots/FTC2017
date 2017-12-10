package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Zorb Ulator on 12/9/2017.
 */
@Autonomous(name="Motor Test", group ="Concept")
public class motorTest extends LinearOpMode{
    public DcMotor leftFront   = null;
    public DcMotor rightFront   = null;
    public DcMotor leftBack   = null;
    public DcMotor rightBack   = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode() {
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        rightFront  = hardwareMap.get(DcMotor.class, "right_front");
        leftBack  = hardwareMap.get(DcMotor.class, "left_back");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back");
        waitForStart();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            leftFront.setPower(1);
            telemetry.addLine("Motor 0");
            telemetry.update();
        }
        leftFront.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            rightFront.setPower(1);
            telemetry.addLine("Motor 1");
            telemetry.update();
        }
        rightFront.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            leftBack.setPower(1);
            telemetry.addLine("Motor 2");
            telemetry.update();
        }
        leftBack.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            rightBack.setPower(1);
            telemetry.addLine("Motor 3");
            telemetry.update();
        }
        rightBack.setPower(0);
    }

}
