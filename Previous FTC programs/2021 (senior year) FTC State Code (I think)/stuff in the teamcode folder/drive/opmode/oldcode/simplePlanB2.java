package org.firstinspires.ftc.teamcode.drive.opmode.oldcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Config
@Autonomous(group = "drive")
public class simplePlanB2 extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startingPosition = new Pose2d(-11.75, -53, Math.toRadians(270));

        Trajectory t1 = drive.trajectoryBuilder(startingPosition)
                .forward(19.5)
                .build();
        Trajectory t2 = drive.trajectoryBuilder(t1.end())
                .lineToLinearHeading(new Pose2d(-11.75,-70.25, Math.toRadians(0)))
                .build();
        Trajectory t3 = drive.trajectoryBuilder(t2.end())
                .lineToLinearHeading(new Pose2d(47.5,-70.5, Math.toRadians(0)))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(t1);
        //Goes up to alliance wobble goal. place freight
        drive.followTrajectory(t2);
        drive.followTrajectory(t3);
        //Gets into warehouse

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}


