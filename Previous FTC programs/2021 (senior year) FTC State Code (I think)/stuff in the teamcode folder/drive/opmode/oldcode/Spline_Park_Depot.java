package org.firstinspires.ftc.teamcode.drive.opmode.oldcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled
public class Spline_Park_Depot extends LinearOpMode {
    //Works!
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        Pose2d poseEstimate = new Pose2d(-40.925,61.75, Math.toRadians(270));
        drive.setPoseEstimate(poseEstimate);
//        Trajectory traj = drive.trajectoryBuilder(new Pose2d(0,0,0))
//                .splineTo(new Vector2d(26.5, -21.075), Math.toRadians(270))
//                .build();
        Trajectory traj = drive.trajectoryBuilder(poseEstimate)
                .splineTo(new Vector2d(-62, 32.25), Math.toRadians(180))
                .build();
//        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-34.25,53,0))
//                .splineTo(new Vector2d(-52.75, 28), 0)
//                .build();

        drive.followTrajectory(traj);

        sleep(2000);

//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj.end(), true)
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build()
        //);
    }
}
