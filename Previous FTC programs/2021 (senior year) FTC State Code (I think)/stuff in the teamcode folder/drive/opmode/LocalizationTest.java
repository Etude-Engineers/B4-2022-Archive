package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    boolean GIVETHISVARIABLEABETTERNAMEBECAUSETHISNAMEISABADNAMEFORAVARIABLETOHAVE = false;
    public TouchSensor Touchy = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Touchy = hardwareMap.get(TouchSensor.class, "Touchy");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            if(drive.Touchy.isPressed()){
                GIVETHISVARIABLEABETTERNAMEBECAUSETHISNAMEISABADNAMEFORAVARIABLETOHAVE = true;
            }else{
                GIVETHISVARIABLEABETTERNAMEBECAUSETHISNAMEISABADNAMEFORAVARIABLETOHAVE = false;
            }

            Pose2d poseEstimate = drive.getPoseEstimate();
            if(gamepad1.x){
                TrajectorySequence toWarehouseEntrance = drive.trajectorySequenceBuilder(poseEstimate)
                        .splineToSplineHeading(new Pose2d(21.5, 66, Math.toRadians(0)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(42,66),Math.toRadians(0))
                        .build();
                drive.followTrajectorySequence(toWarehouseEntrance);
            }
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Alpha 1: ", drive.Color.alpha());
            telemetry.addData("Alpha 2: ", drive.Color2.alpha());
            telemetry.addData("green 1: ", drive.Color.green());
            telemetry.addData("green 2: ", drive.Color2.green());
            telemetry.addData("red 1: ", drive.Color.red());
            telemetry.addData("red 2: ", drive.Color2.red());
            telemetry.addData("Intake Alpha: ", drive.IntakeColor.alpha());
            telemetry.addData("touchy touched?: ", GIVETHISVARIABLEABETTERNAMEBECAUSETHISNAMEISABADNAMEFORAVARIABLETOHAVE);
            telemetry.addData("1 Touch Sensor: ", drive.IntakeTouch1.isPressed());
            telemetry.addData("2nd Touch Sensor: ", drive.IntakeTouch1.isPressed());
            telemetry.addData("Distance Sensor: ", drive.Distance.getDistance(DistanceUnit.INCH));

            telemetry.update();

        }
    }
}
