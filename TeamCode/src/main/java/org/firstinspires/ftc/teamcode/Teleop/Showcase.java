package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Showcase")
public class Showcase extends LinearOpMode {

    private DcMotor LB; // 0C
    private DcMotor LF; // 1C
    private DcMotor RB; // 2C
    private DcMotor RF; // 3C
    private DcMotor Arm;

    // Drive power values
    double lbPower;
    double lfPower;
    double rbPower;
    double rfPower;
    double arm_vel = 1500;



    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Hardware mapping
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        Arm = hardwareMap.get(DcMotor.class, "Arm");

        // Motor directions
        LB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);
        Arm.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            armFunction();
            
            // Mecanum drive calculations
            double forward = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x;
            double turn    = -gamepad1.right_stick_x;



            lbPower = forward - strafe + turn;
            lfPower = forward + strafe + turn;
            rbPower = forward + strafe - turn;
            rfPower = forward - strafe - turn;

            // Normalize
            double max = Math.max(Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                    Math.max(Math.abs(lbPower), Math.abs(rbPower)));

            if (max > 1.0) {
                lbPower /= max;
                lfPower /= max;
                rbPower /= max;
                rfPower /= max;
            }

            // Set drive motor power
            LB.setPower(lbPower);
            LF.setPower(lfPower);
            RB.setPower(rbPower);
            RF.setPower(rfPower);



            // Telemetry (Driver Station + Dashboard)
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("LB:", lbPower);
            telemetry.addData("LF:", lfPower);
            telemetry.addData("RB:", rbPower);
            telemetry.addData("RF:", rfPower);
            telemetry.update();
        }
    }

    private void armFunction() {
        if (gamepad1.dpad_up)
            Arm.setPower(arm_vel);
        else if (gamepad1.dpad_down)
            Arm.setPower(-arm_vel);
        else
            Arm.setPower(0);
    }
}
