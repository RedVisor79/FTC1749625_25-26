package org.firstinspires.ftc.teamcode.Auto.newAuto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "DRIVE_HARDWARE_TEST")
public class DRIVE_HARDWARE_TEST extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Reverse right side (very common for mecanum)
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("READY - PRESS PLAY");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // =========================
        // FORWARD
        // =========================
        telemetry.addLine("FORWARD");
        telemetry.update();

        leftFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightFront.setPower(0.5);
        rightBack.setPower(0.5);

        sleep(2000);

        // STOP
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        sleep(1000);

        // =========================
        // STRAFE LEFT
        // =========================
        telemetry.addLine("STRAFE LEFT");
        telemetry.update();

        leftFront.setPower(-0.5);
        leftBack.setPower(0.5);
        rightFront.setPower(0.5);
        rightBack.setPower(-0.5);

        sleep(2000);

        // STOP
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        sleep(1000);

        // =========================
        // TURN RIGHT
        // =========================
        telemetry.addLine("TURN RIGHT");
        telemetry.update();

        leftFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightFront.setPower(-0.5);
        rightBack.setPower(-0.5);

        sleep(2000);

        // FINAL STOP
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        telemetry.addLine("TEST COMPLETE");
        telemetry.update();
        sleep(3000);
    }
}
