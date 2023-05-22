//package org.firstinspires.ftc.teamcode.drive.opmode.teleop.StateBasedControl;
//
//import com.acmerobotics.roadrunner.drive.Drive;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.drive.HardwareTeleop;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//@Disabled
//@TeleOp(name="StateBasedTeleop", group="Pushbot")
//public class StateBasedTeleop extends LinearOpMode {
//
//    DriveSubsystem driveSubsystem = new DriveSubsystem();
//    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
//    LiftSubsystem liftSubsystem = new LiftSubsystem();
//
//    public static double DrivePower = 1;
//
//    @Override
//    public void runOpMode() {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        while (opModeIsActive()) {
//            drive.update();
//
//            Pose2d poseEstimate = drive.getPoseEstimate();
//            if(gamepad1.x)
//            {
//                TrajectorySequence DriveToHub = drive.trajectorySequenceBuilder(poseEstimate) //Goes to Hub:
//                        .strafeLeft(12)
//                        .splineToConstantHeading(new Vector2d(-12.75, 39.25),Math.toRadians(90)) //seems like 1.5 inch off
//                        .build();
//                driveSubsystem.RunTrajectorySequence(DriveToHub);
//
//                driveSubsystem.driveState = DriveSubsystem.DriveState.Automated;
//
//            }
//            else if(gamepad1.y)
//            {
//                Trajectory DrivetoGamepeice = drive.trajectoryBuilder(poseEstimate) //Goes to Scan:
//                .lineToConstantHeading(new Vector2d(6.625, 45.675)) //y was like 4 inches too little, x was like an inch too little
//                .build();
////                driveSubsystem.RunTrajectory(DrivetoGamepeice);
//
//                driveSubsystem.driveState = DriveSubsystem.DriveState.Automated;
//            }
//            driveSubsystem.RunSubsystem(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
//            intakeSubsystem.RunSubsystem();
//            liftSubsystem.RunSubsystem();
//        }
//
//    }
//}
