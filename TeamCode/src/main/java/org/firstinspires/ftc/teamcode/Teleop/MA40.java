package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

@Config
@TeleOp(name = "MA40", group = "Teleop")
public class MA40 extends LinearOpMode {

    /* =========================
       SHOOTER
       ========================= */
    DcMotorEx shooterLeft;
    DcMotorEx shooterRight;

    public static double SHOOT_CLOSE_VEL = 1450;
    public static double SHOOT_FAR_VEL   = 1800;

    int shooterDistance = 1; // 1 = close, 2 = far

    /* =========================
       INTAKE
       ========================= */
    DcMotorEx intake;
    public static double INTAKE_VEL = 2000;

    /* =========================
       HOOD
       ========================= */
    Servo hoodL, hoodR;
    double hoodPos = 0.0;
    public static double HOOD_INCREMENT = 0.02;

    @Override
    public void runOpMode() {

        /* =========================
           ROAD RUNNER DRIVE
           ========================= */
        Pose2d startPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        /* =========================
           HARDWARE INIT (NON-DRIVE)
           ========================= */
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "LS");
        shooterRight = hardwareMap.get(DcMotorEx.class, "RS");

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        hoodL = hardwareMap.get(Servo.class, "hoodL");
        hoodR = hardwareMap.get(Servo.class, "hoodR");
        hoodR.setDirection(Servo.Direction.REVERSE);

        hoodL.setPosition(hoodPos);
        hoodR.setPosition(hoodPos);

        telemetry.addLine("RR TeleOp Ready");
        telemetry.update();

        /* =========================
           WAIT FOR START
           ========================= */
        waitForStart();

        /* =========================
           TELEOP LOOP
           ========================= */
        while (opModeIsActive() && !isStopRequested()) {

            /* ---- REQUIRED FOR ODOMETRY ---- */
            drive.updatePoseEstimate();

            /* =========================
               DRIVE (ROBOT-CENTRIC)
               ========================= */
            double forward = -gamepad1.left_stick_y;
            double strafe  = gamepad1.left_stick_x;
            double turn    = gamepad1.right_stick_x;

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(forward, strafe),
                            turn
                    )
            );

            /* =========================
               SHOOTER
               ========================= */
            if (gamepad1.dpad_up) shooterDistance = 1;
            if (gamepad1.dpad_left) shooterDistance = 2;

            if (gamepad1.a) {
                double vel = (shooterDistance == 1)
                        ? SHOOT_CLOSE_VEL
                        : SHOOT_FAR_VEL;

                shooterLeft.setVelocity(vel);
                shooterRight.setVelocity(vel);
            }

            if (gamepad1.b) {
                shooterLeft.setVelocity(0);
                shooterRight.setVelocity(0);
            }

            /* =========================
               INTAKE
               ========================= */
            if (gamepad1.right_trigger > 0.2) {
                intake.setVelocity(INTAKE_VEL);
            } else if (gamepad1.left_trigger > 0.2) {
                intake.setVelocity(-INTAKE_VEL);
            } else {
                intake.setVelocity(0);
            }

            /* =========================
               HOOD
               ========================= */
            if (gamepad1.right_bumper) {
                hoodPos = Math.min(hoodPos + HOOD_INCREMENT, 1.0);
            }
            if (gamepad1.left_bumper) {
                hoodPos = Math.max(hoodPos - HOOD_INCREMENT, 0.0);
            }

            hoodL.setPosition(hoodPos);
            hoodR.setPosition(hoodPos);

            /* =========================
               TELEMETRY
               ========================= */
            Pose2d pose = drive.localizer.getPose();
            telemetry.addData("X", "%.1f", pose.position.x);
            telemetry.addData("Y", "%.1f", pose.position.y);
            telemetry.addData("Heading (deg)", "%.1f",
                    Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("Shooter Mode",
                    shooterDistance == 1 ? "CLOSE" : "FAR");
            telemetry.addData("Hood Pos", "%.2f", hoodPos);
            telemetry.update();
        }
    }
}
