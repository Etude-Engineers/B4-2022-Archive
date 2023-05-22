package org.firstinspires.ftc.teamcode.drive.opmode.oldcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Config
@Autonomous(group = "drive")
public class RoadRunnerWarehouseFarPark extends LinearOpMode{
//Simple Plan #?

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startingPosition = new Pose2d(0, 0, Math.toRadians(0));

//        Trajectory t1 = drive.trajectoryBuilder(startingPosition)
//                .forward(21.625)
//                .build();
//        Trajectory t1_5 = drive.trajectoryBuilder(t1.end())
//                .strafeRight(7.4)
//                .build();
//        Trajectory t2 = drive.trajectoryBuilder(t1_5.end())
//                .splineTo(new Vector2d(41.25, -25.45), Math.toRadians(0))
//                .build();
//        Trajectory t2_5 = drive.trajectoryBuilder(t2.end())
//                .splineTo(new Vector2d(41.25, -25.45), Math.toRadians(90))
//                .build();
//        Trajectory t3 = drive.trajectoryBuilder(t2_5.end())
//                .strafeRight(7.45)
//                .build();
//        Trajectory t4 = drive.trajectoryBuilder(t3.end())
//                .lineToLinearHeading(new Pose2d(-21.75,-70.5, Math.toRadians(0)))
//                .build();
//        Trajectory t5 = drive.trajectoryBuilder(t4.end())
//                .lineToLinearHeading(new Pose2d(47.5,-70.5, Math.toRadians(0)))
//                .build();

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startingPosition)
                .forward(23)
                .strafeRight(7.4)
                .splineTo(new Vector2d(41.25, -25.45), Math.toRadians(0))
                .turn(Math.toRadians(90))
                .splineTo(new Vector2d(49.7, -25.45), Math.toRadians(90))
                //.strafeRight(7.45)
                .build();
        waitForStart();

        if (isStopRequested()) return;

//        drive.followTrajectory(t1);
//        drive.followTrajectory(t1_5);
//        drive.followTrajectory(t2);
//        drive.turn(Math.toRadians(90));
//        drive.followTrajectory(t3);
        drive.followTrajectorySequence(trajSeq1);



        //Detect if TSE is there. If so, DO NOT skip next. As it is set up, that will mess up t3. It can be fixed, but we really don't have the time right now. Can still skip the scan, but do NOT skip the movement
        //drive.followTrajectory(t2);
        //Detect if TSE is there
        //drive.followTrajectory(t3);
        //Place block based on the earlier detections
        //drive.followTrajectory(t4);
        //drive.followTrajectory(t5); //gets to edge of where freight should be. Stop here to park, add more to pick stuff up, or move to the right to get out of the way (if the partner is also going through the crack)


        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}