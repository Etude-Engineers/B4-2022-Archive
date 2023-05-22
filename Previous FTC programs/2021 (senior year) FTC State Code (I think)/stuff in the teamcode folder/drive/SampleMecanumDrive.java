package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {
    public double doorUp = 0.75;
    public double doorDown = 0.5;

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(2, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

    public static double LATERAL_MULTIPLIER = 1.066201236;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    //Robot Components:

    //Scoring Freight:
    //Intake:
    public DcMotor Intake = null;
    public Servo IntakeDoor = null;

    //Box:
    public Servo Indexer = null;
    //Lift:
    public DcMotor Lift = null;

    public Servo CapstoneArm = null;
    public Servo CapstoneHand = null;

    //Engame:
    public DcMotor Spinner = null;
    public DcMotor TapeMotor = null;

    //Game Piece Servos:
//  public CRServo tapetheta = null;
//  public CRServo tapephi = null;
    public Servo tapetheta = null;
    public Servo tapephi = null;
//    public Servo tapephi2 = null;

    //Sensors:
    //Lift:
    public TouchSensor Touchy = null;

    //Intake:
    public DistanceSensor Distance = null;
    public TouchSensor IntakeTouch1 = null;
    public TouchSensor IntakeTouch2 = null;

    //Auto:
    public ColorSensor Color = null;
    public ColorSensor Color2 = null;
    public ColorSensor IntakeColor = null;
    public double DrivePower = 1;

    public RevBlinkinLedDriver blinkinLedDriver = null;


    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        //Scoring Freight:
        //Intake:
        Intake  = hardwareMap.get(DcMotor.class, "Intake");
        IntakeDoor = hardwareMap.get(Servo.class, "IntakeDoor");
        //Box:
        Indexer = hardwareMap.get(Servo.class,"Indexer");

        Lift = hardwareMap.get(DcMotor.class,"Lift");
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);

        //Endgame:
        Spinner = hardwareMap.get(DcMotor.class,"Spinner");

        //Sensors:
        Touchy = hardwareMap.get(TouchSensor.class, "Touchy");
        IntakeTouch1 = hardwareMap.get(TouchSensor.class, "IntakeTouch1");
        IntakeTouch2 = hardwareMap.get(TouchSensor.class, "IntakeTouch2");
        Color = hardwareMap.get(ColorSensor.class,"Color");
        Color2 = hardwareMap.get(ColorSensor.class,"Color2");
        IntakeColor = hardwareMap.get(ColorSensor.class, "IntakeColor");
        Distance = hardwareMap.get(DistanceSensor.class,"Distance");

        //Tape Servos:
//      tapetheta = hardwareMap.get(CRServo.class,"tapetheta");
//      tapephi = hardwareMap.get(CRServo.class,"tapephi");
        tapetheta = hardwareMap.get(Servo.class,"tapetheta");
        tapephi = hardwareMap.get(Servo.class,"tapephi");
//        tapephi2 = hardwareMap.get(Servo.class,"tapephi2");
        CapstoneArm = hardwareMap.get(Servo.class,"CapstoneArm");
        CapstoneHand = hardwareMap.get(Servo.class,"CapstoneHand");

        TapeMotor = hardwareMap.get(DcMotor.class,"TapeMotor");

        //Drive Motors:
        leftFront = hardwareMap.get(DcMotorEx.class, "FL");
        leftRear = hardwareMap.get(DcMotorEx.class, "BL");
        rightRear = hardwareMap.get(DcMotorEx.class, "BR");
        rightFront = hardwareMap.get(DcMotorEx.class, "FR");

        //Initialize Components:
        Lift.setDirection(DcMotor.Direction.FORWARD);
        Spinner.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.REVERSE);


        //Tape Servos:
//        tapetheta.setPower(0);
//        tapephi.setPower(0);
        tapetheta.setPosition(0);
        tapephi.setPosition(0);
        TapeMotor.setPower(0);
        Intake.setPower(0);
        Indexer.setPosition(0.7); //Middle Position
        Lift.setPower(0);
        Spinner.setPower(0);
        IntakeDoor.setPosition(doorUp);//Up

        tapetheta.setPosition(0.5);
        tapephi.setPosition(0.5);
//        tapephi2.setPosition(0.5);

        CapstoneArm.setPosition(0.12);
        CapstoneHand.setPosition(0.6);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // Done: reverse any motors using DcMotor.setDirection()
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
//        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
//        rightRear.setDirection(DcMotorEx.Direction.REVERSE);
        //leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        // Done: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return 0;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // TODO: This must be changed to match your configuration
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // The positive x axis points toward the USB port(s)
        //
        // Adjust the axis rotation rate as necessary
        // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
        // flat on a surface

        // To work around an SDK bug, use -zRotationRate in place of xRotationRate
        // and -xRotationRate in place of zRotationRate (yRotationRate behaves as
        // expected). This bug does NOT affect orientation.
        //
        // See https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/251 for details.
        return (double) -imu.getAngularVelocity().xRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
    public void setDriveForMecanum(Mecanum.Motion motion) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
        leftFront.setPower(wheels.frontLeft * DrivePower);
        rightFront.setPower(wheels.frontRight * DrivePower);
        leftRear.setPower(wheels.backLeft * DrivePower);
        rightRear.setPower(wheels.backRight * DrivePower);
    }
//    private void Lift1() {
//        boolean stopped = false;
////        telemetry.addData("Starting:", "Lift to First Level");
////        telemetry.update();
////            if(gamepad2.right_bumper)
////            {
////                stopped=true;
////            }
//
//        Indexer.setPosition(BoxLiftingPosition);
//        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        double liftTargetPos = 560;
//        Lift.setPower(.6);
//        while (Lift.getCurrentPosition() < liftTargetPos && !stopped) {
////            telemetry.addData("Height:", drive.Lift.getCurrentPosition());
////            telemetry.update();
//        }
//        Lift.setPower(0);
//        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        telemetry.addData("Ready to Dump", "Ready to Dump");
////        telemetry.update();
//
//    }
//
//    private void Lift2() {
//        boolean stopped = false;
////        telemetry.addData("Starting:", "Lift to First Level");
////        telemetry.update();
////            if(gamepad2.right_bumper)
////            {
////                stopped=true;
////            }
//
//        Indexer.setPosition(BoxLiftingPosition);
//        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        double liftTargetPos = 1120;
//        Lift.setPower(.6);
//        while (Lift.getCurrentPosition() < liftTargetPos && !stopped) {
////            telemetry.addData("Height:", drive.Lift.getCurrentPosition());
////            telemetry.update();
//        }
//        Lift.setPower(0);
//        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        telemetry.addData("Ready to Dump", "Ready to Dump");
////        telemetry.update();
//
//    }
//
//
//    private void Lift3() {
//        boolean stopped = false;
////        telemetry.addData("Starting:", "Lift to 3rd Level");
////        telemetry.update();
////            if(gamepad2.left_bumper)
////            {
////                stopped=true;
////                break;
////            }
//        Indexer.setPosition(BoxLiftingPosition);
//        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Lift.setPower(.6);
//        Lift.setTargetPosition(1680);
//        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        while (Lift.isBusy() && !stopped) {
////            telemetry.addData("Height:", drive.Lift.getCurrentPosition());
//           // telemetry.update();
//        }
//        Lift.setPower(0);
//        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        telemetry.addData("Ready to Dump", "Ready to Dump");
////        telemetry.update();
//
//    }

//    private void DumpAndLower3() {
//        boolean stopped = false;
//        while (!stopped) {
//            telemetry.addData("Starting:", "Dump & Lower");
//            telemetry.update();
////            if (gamepad2.dpad_down) {
////                stopped = true;
////                break;
////            }
//            //position while dumped
//            drive.Indexer.setPosition(BoxDumpingPosition);
//            sleep(700);
//            drive.Indexer.setPosition(BoxLiftingPosition);
//            drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            drive.Lift.setPower(.6);
//            drive.Lift.setTargetPosition(-3 * CountsPerlevel);
//            drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (drive.Lift.isBusy() && !stopped) {
//                telemetry.addData("Height:", drive.Lift.getCurrentPosition());
//                telemetry.update();
//                //wait for it to finish
////                if(gamepad2.dpad_down)
////                {
////                    stopped=true;
////                    break;
////                }
//            }
//            drive.Lift.setPower(0);
//            drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            drive.Indexer.setPosition(BoxStartingPosition);
//            telemetry.addData("Ready to Reload", "");
//            telemetry.update();
//        }
//    }
//
//    private void DumpAndLower2() {
//        boolean stopped = false;
//        while (!stopped) {
////            telemetry.addData("Starting:", "Dump & Lower");
////            telemetry.update();
////            if (gamepad2.dpad_down) {
////                stopped = true;
////                break;
////            }
//            //position while dumped
//            Indexer.setPosition(BoxDumpingPosition);
//            sleep(700);
//            Indexer.setPosition(BoxLiftingPosition);
//            Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            Lift.setPower(.6);
//            Lift.setTargetPosition(-CountsPerlevel * 2);
//            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (Lift.isBusy() && !stopped) {
//                telemetry.addData("Height:", drive.Lift.getCurrentPosition());
//                telemetry.update();
//                //wait for it to finish
////                if(gamepad2.dpad_down)
////                {
////                    stopped=true;
////                    break;
////                }
//            }
//            Lift.setPower(0);
//            Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            Indexer.setPosition(BoxStartingPosition);
////            telemetry.addData("Ready to Reload", "");
////            telemetry.update();
//        }
//    }
//
//    private void DumpAndLower1() {
//        boolean stopped = false;
//        while (!stopped) {
////            telemetry.addData("Starting:", "Dump & Lower");
////            telemetry.update();
////            if (gamepad2.dpad_down) {
////                stopped = true;
////                break;
////            }
//            //position while dumped
//            Indexer.setPosition(BoxDumpingPosition);
//            sleep(700);
//            Indexer.setPosition(BoxLiftingPosition);
//            Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            Lift.setPower(.6);
//            drive.Lift.setTargetPosition(-CountsPerlevel);
//            drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (drive.Lift.isBusy() && !stopped) {
//                telemetry.addData("Height:", drive.Lift.getCurrentPosition());
//                telemetry.update();
//                //wait for it to finish
////                if(gamepad2.dpad_down)
////                {
////                    stopped=true;
////                    break;
////                }
//            }
//            drive.Lift.setPower(0);
//            drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            drive.Indexer.setPosition(BoxStartingPosition);
//            telemetry.addData("Ready to Reload", "");
//            telemetry.update();
//        }
//    }

//    public void Scan() {
//        if(Color.green()>100)
//        {
//            level = 3;
//        }
//        else if (Color2.green()>100)
//        {
//            level = 2;
//        }
//        else
//        {
//            level = 1;
//        }
//    }
}
