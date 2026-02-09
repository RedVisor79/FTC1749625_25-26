package org.firstinspires.ftc.teamcode.Teleop;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//don't use this this was a WIP telop

@Config
@Autonomous(name = "HaloReach", group = "Teleop")
public class HaloReach extends LinearOpMode {

    public class Shooter {
        private DcMotorEx LSX; // 0E
        private DcMotorEx RSX; // 1E
        private int distance=1;

        public Shooter(HardwareMap hardwareMap) {

            LSX = hardwareMap.get(DcMotorEx.class, "LS");
            LSX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LSX.setDirection(DcMotorSimple.Direction.FORWARD);

            RSX = hardwareMap.get(DcMotorEx.class, "RS");
            RSX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RSX.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class ShootClose implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    LSX.setVelocity(1450);
                    RSX.setVelocity(1450);
                    initialized = true;
                }
                return initialized;
            }
        }

        public Action shootClose() {
            return new ShootClose();
        }

        public class ShootFar implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    LSX.setVelocity(1800);
                    RSX.setVelocity(1800);
                    initialized = true;
                }
                return initialized;
            }
        }
        public Action shooterFar() {
            return new ShootFar();
        }

        public class ShootStop implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    LSX.setVelocity(0);
                    RSX.setVelocity(0);
                    initialized = true;
                }
                return initialized;
            }
        }
        public Action shootStop() {
            return new ShootStop();
        }

        public class DistanceSetClose implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    distance = 1;
                    initialized = true;
                }
                return initialized;
            }
        }

        public Action distanceSetClose() {
            return new DistanceSetClose();
        }

        public class DistanceSetFar implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    distance = 2;
                    initialized = true;
                }
                return initialized;
            }
        }

        public Action distanceSetFar() {
            return new DistanceSetFar();
        }
    }


    public class Intaker {
        private DcMotorEx IntakeMotor; // 2E

        public Intaker(HardwareMap hardwareMap) {
            IntakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
            IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            IntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class IntakeIn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    IntakeMotor.setVelocity(2000);
                    initialized = true;
                }
                return initialized;
            }
        }

        public Action intakeIn() {
            return new IntakeIn();
        }

        public class IntakeOut implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    IntakeMotor.setVelocity(-2000);
                    initialized = true;
                }
                return initialized;
            }
        }

        public Action intakeOut() {
            return new IntakeOut();
        }

        public class IntakeOff implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    IntakeMotor.setVelocity(0);
                    initialized = true;
                }
                return initialized;
            }
        }
        public Action intakeOff() {
            return new IntakeOff();
        }

    }

    public class Hood {
        private Servo leftServo;
        private Servo rightServo;
        final double INCREMENT = 0.02;

        // Servo max angle in degrees (your calibrated max)
        final double MAX_ANGLE = 200.0;

        // Current positions (0–1 range)
        double leftPos;
        double rightPos;

        public Hood(HardwareMap hardwareMap) {
            // CHANGE THESE TO MATCH YOUR ROBOT CONFIG
            leftServo  = hardwareMap.get(Servo.class, "hoodL");
            rightServo = hardwareMap.get(Servo.class, "hoodR");

            // Reverse one servo because they face each other
            rightServo.setDirection(Servo.Direction.REVERSE);

            // RESET zero position at start
            leftPos = 0.0;    // fully out
            rightPos = 0.0;   // fully out
            leftServo.setPosition(leftPos);
            rightServo.setPosition(rightPos);
        }

        public class HoodUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // Move IN
                    leftPos  = Math.min(leftPos + INCREMENT, 1.0);
                    rightPos = Math.min(rightPos + INCREMENT, 1.0);

                    leftServo.setPosition(leftPos);
                    rightServo.setPosition(rightPos);

                    // Telemetry: show 0–1 and degrees
                    telemetry.addData("Left Position", "%.2f (%.1f deg)", leftPos, leftPos * MAX_ANGLE);
                    telemetry.addData("Right Position", "%.2f (%.1f deg)", rightPos, rightPos * MAX_ANGLE);
                    telemetry.update();
                }
                return initialized;
            }
        }
        public Action hoodUp() {
            return new HoodUp();
        }

        public class HoodDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // Move OUT
                    leftPos  = Math.max(leftPos - INCREMENT, 0.0);
                    rightPos = Math.max(rightPos - INCREMENT, 0.0);

                    leftServo.setPosition(leftPos);
                    rightServo.setPosition(rightPos);

                    // Telemetry: show 0–1 and degrees
                    telemetry.addData("Left Position", "%.2f (%.1f deg)", leftPos, leftPos * MAX_ANGLE);
                    telemetry.addData("Right Position", "%.2f (%.1f deg)", rightPos, rightPos * MAX_ANGLE);
                    telemetry.update();
                }
                return initialized;
            }
        }
        public Action hoodDown() {
            return new HoodDown();
        }
    }

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Shooter launcher = new Shooter(hardwareMap);
        Intaker intake = new Intaker(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();

        // vision here that outputs position
        int visionOutputPosition = 1;

        /*TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();
        */
        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(claw.closeClaw());

        runtime.reset();
        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            if (gamepad1.aWasPressed()&&launcher.LSX.getVelocity()!=0){
                if (launcher.distance == 1) launcher.shootClose();
                if (launcher.distance == 2) launcher.shooterFar();
            }
            if (gamepad1.bWasPressed()&&launcher.LSX.getVelocity()!=0){
                launcher.shootStop();
            }
            if (gamepad1.right_trigger>0&&intake.IntakeMotor.getVelocity()!=0){
                intake.intakeIn();
            }
            if (gamepad1.left_trigger>0&&intake.IntakeMotor.getVelocity()!=0){
                intake.intakeOut();
            }
            if (gamepad1.dpad_left) launcher.distanceSetFar();
            if (gamepad1.dpad_up) launcher.distanceSetClose();
            else {
                intake.intakeOff();
            }
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        /*if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }*/

        Actions.runBlocking(
                new SequentialAction(
                )
        );
    }
}