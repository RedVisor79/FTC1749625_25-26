package org.firstinspires.ftc.teamcode.Auto.newAuto;
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

@Config
@Autonomous(name = "testAuto", group = "Autonomous")
public class testAuto extends LinearOpMode {

    public class Shooter {
        private DcMotorEx LSX; // 0E
        private DcMotorEx RSX; // 1E

        public double VEL = 1800;

        public Shooter(HardwareMap hardwareMap) {
            LSX  = hardwareMap.get(DcMotorEx.class, "LS");
            RSX = hardwareMap.get(DcMotorEx.class, "RS");

            LSX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RSX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LSX.setDirection(DcMotorSimple.Direction.FORWARD);
            RSX.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class Shoot implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    LSX.setVelocity(VEL);
                    RSX.setVelocity(VEL);
                    initialized = true;
                }
                return initialized;
            }
        }

        public Action ShootClose() {
            return new Shoot();
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
        public Action ShootStop() {
            return new ShootStop();
        }
    }

    public class Intaker {
        private DcMotorEx IntakeMotor; // 2E

        public Intaker(HardwareMap hardwareMap) {
            IntakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
            IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            IntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class intakeOn implements Action {
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

        public Action intakeOn() {
            return new intakeOn();
        }

        public class intakeOff implements Action {
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
            return new intakeOff();
        }

    }

    public class Hood {
        Servo hoodL;
        Servo hoodR;
        public double hoodPos;

        public Hood(HardwareMap hardwareMap) {
            hoodL = hardwareMap.get(Servo.class, "hoodL");
            hoodR = hardwareMap.get(Servo.class, "hoodR");
            hoodR.setDirection(Servo.Direction.REVERSE);
            hoodR.setDirection(Servo.Direction.FORWARD);
        }

        public class hoodSet implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    hoodL.setPosition(hoodPos);
                    hoodR.setPosition(hoodPos);
                    initialized = true;
                }
                return initialized;
            }
        }

        public Action hoodSet() {
            return new hoodSet();
        }
        
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Shooter claw = new Shooter(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
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

        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(claw.closeClaw());


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        trajectoryActionCloseOut
                )
        );
    }
}