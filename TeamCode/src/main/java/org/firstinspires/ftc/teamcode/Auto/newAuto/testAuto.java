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
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action ShootClose() {
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
            }
        }
        public Action ShooterFar() {
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
            }
        }
        public Action intakeOff() {
            return new intakeOff();
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
                        //lift.liftUp(),
                        //claw.openClaw(),
                        //lift.liftDown(),
                        trajectoryActionCloseOut
                )
        );
    }
}