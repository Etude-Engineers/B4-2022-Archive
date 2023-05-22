//package org.firstinspires.ftc.teamcode.drive.opmode.teleop.StateBasedControl;
//
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.drive.HardwareTeleop;
//import org.firstinspires.ftc.teamcode.drive.Mecanum;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//public class DriveSubsystem extends LinearOpMode {
//
//    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//    }
//    //Initiates to driving with Mecanum:
//    DriveState driveState = DriveState.Mecanum;
//    TrajectorySequence TrajectorySequence;
//
//    public DriveSubsystem()
//    {
//    }
//
//
//
//    public enum DriveState
//    {
//        Mecanum,
//        Automated
//    }
//
//    public void RunSubsystem(double LeftY, double LeftX, double RightX)
//    {
//
//        switch (driveState)
//        {
//            case Mecanum:
//                setDriveForMecanum(Mecanum.joystickToMotion(
//                        LeftX, -LeftY,
//                        -RightX, 0));
//                break;
//
//            case Automated:
//
//                drive.followTrajectorySequenceAsync(TrajectorySequence);
//                break;
//        }
//    }
//    public void RunTrajectorySequence(TrajectorySequence trajectorysequence)
//    {
//        TrajectorySequence = trajectorysequence;
//    }
////    public void RunTrajectory(Trajectory trajectory)
////    {
////        TrajectorySequence = trajectorysequence;
////    }
//
//    private void setDriveForMecanum(Mecanum.Motion motion) {
//        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
//        drive.leftFront.setPower(wheels.frontLeft * StateBasedTeleop.DrivePower);
//        drive.rightFront.setPower(wheels.frontRight * StateBasedTeleop.DrivePower);
//        drive.leftRear.setPower(wheels.backLeft * StateBasedTeleop.DrivePower);
//        drive.rightRear.setPower(wheels.backRight * StateBasedTeleop.DrivePower);
//    }
//}
