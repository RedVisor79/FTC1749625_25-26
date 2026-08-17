package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Showcase")
public class Showcase extends LinearOpMode {

    private DcMotor LB;
    private DcMotor LF;
    private DcMotor RB;
    private DcMotor RF;
    private DcMotor Arm;
    private DcMotor Claw;

    // Drive power values
    double lbPower;
    double lfPower;
    double rbPower;
    double rfPower;

    //==========================
    // Arm PID Variables
    //==========================

    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.0002;

    public static int armTarget = 0;

    private double integral = 0;
    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();

    private boolean lastUp = false;
    private boolean lastDown = false;

    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();

        // Hardware mapping
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Claw = hardwareMap.get(DcMotor.class, "Claw");

        // Motor directions
        LB.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.REVERSE);
        Arm.setDirection(DcMotor.Direction.FORWARD);

        // Brake motors
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Drive encoders
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Arm encoder
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();
        pidTimer.reset();

        while (opModeIsActive()) {

            armFunction();


            //==========================
            // Mecanum Drive
            //==========================

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            lbPower = forward - strafe + turn;
            lfPower = forward + strafe + turn;
            rbPower = forward + strafe - turn;
            rfPower = forward - strafe - turn;

            double max = Math.max(
                    Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                    Math.max(Math.abs(lbPower), Math.abs(rbPower))
            );

            if (max > 1.0) {
                lbPower /= max;
                lfPower /= max;
                rbPower /= max;
                rfPower /= max;
            }

            LB.setPower(lbPower);
            LF.setPower(lfPower);
            RB.setPower(rbPower);
            RF.setPower(rfPower);

            telemetry.addData("Run Time", runtime);
            telemetry.addData("Arm Target", armTarget);
            telemetry.addData("Arm Position", Arm.getCurrentPosition());
            telemetry.addData("Arm Power", Arm.getPower());
            telemetry.update();
        }
    }

    private void armFunction() {

        // Move target up/down one time per button press
        if (gamepad1.dpad_up && !lastUp) {
            armTarget += 75;
        }

        if (gamepad1.dpadUpWasReleased() && !lastDown) {
            armTarget -= 75;
        }

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpadUpWasReleased();

        // Safety limits (adjust for your robot)
        armTarget = Math.max(0, Math.min(3000, armTarget));

        int currentPosition = Arm.getCurrentPosition();

        double error = armTarget - currentPosition;

        double dt = pidTimer.seconds();
        pidTimer.reset();

        if (dt > 0) {
            integral += error * dt;
        }

        double derivative = dt > 0 ? (error - lastError) / dt : 0;

        double output =
                (kP * error) +
                        (kI * integral) +
                        (kD * derivative);

        // Limit motor power
        output = Math.max(-1.0, Math.min(1.0, output));

        Arm.setPower(output);

        lastError = error;
    }


    }