package org.firstinspires.ftc.teamcode.drive.opmode.oldcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Disabled
@Autonomous(group = "drive")
public class BlueDepotFullAuto extends LinearOpMode {
//Simple Plan #?


    final double BoxStartingPosition = 0.55;
    final double BoxLiftingPosition = 0.7;
    final double BoxDumpingPosition = 1;
    int level = 1;
    ElapsedTime lifttime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        //We will have Sensors on the back:
        //Pose2d startingPosition = new Pose2d(-40.875,61.75, Math.toRadians(90)); //If we Scan first

        //Line up the right side of the drive train bar with the seam closest to spinner
        Pose2d startingPosition = new Pose2d(-56.325, 61.75, Math.toRadians(90)); //If we Spin First, off by 1.5in
        drive.setPoseEstimate(startingPosition);
        drive.Color.enableLed(true);
        drive.Color2.enableLed(true);

        //Spinning the Duck
        Trajectory t1 = drive.trajectoryBuilder(startingPosition)
                .lineToConstantHeading(new Vector2d(-62.25, 56.75)) //Goes like an inch less then it should, I think becuase it's so close
                .build();

        //2 Parts, first moves away from wall, 2nd moves straight to the position
        // ^ That is an example of what a comment should be, but it should have been deleted when it no longer became true.
        TrajectorySequence t2 = drive.trajectorySequenceBuilder(t1.end())
                .lineToConstantHeading(new Vector2d(-40.375, 48.675)) //y was like 4 inches too little, x was like an inch too little
                .build();


        //Placing Block:
        TrajectorySequence t3 = drive.trajectorySequenceBuilder(t2.end())
                .strafeRight(25)
                .lineToConstantHeading(new Vector2d(-14.25, 41.75)) //x was like an inch too little
                .build();

        //Parking in Depot
        TrajectorySequence t4 = drive.trajectorySequenceBuilder(t3.end())
                .lineTo(new Vector2d(-40.25,55.75))
                .lineToSplineHeading(new Pose2d(-64.5, 29, Math.toRadians(180))) //Center from x seems to be 2 inches off, Center for y seems to be 6 inches from what should have been center
                .build();
        TrajectorySequence special = drive.trajectorySequenceBuilder(t3.end())
                .lineToSplineHeading(new Pose2d(-56, 55,Math.toRadians(180))) //x was like an inch too little
                .lineToSplineHeading(new Pose2d(-64.5, 29, Math.toRadians(180))) //Center from x seems to be 2 inches off, Center for y seems to be 6 inches from what should have been center
                .build();

        waitForStart();

        if (isStopRequested()) return;
        sleep(400);
        drive.followTrajectory(t1);
        drive.Spinner.setPower(0.3);
        telemetry.addData("Color:",drive.Color.green());
        telemetry.update();
        sleep(3000); //Spin
        drive.Spinner.setPower(0);
        drive.followTrajectorySequence(t2);
//        if(drive.Color.green()>81) //Needs to be at least >93
//        {
//            level = 3;
//            telemetry.addData("Level:", level);
//            telemetry.addData("Color:", drive.Color.green());
//            telemetry.addData("Color2:", drive.Color2.green());
//            telemetry.update();
//        }
//        else if (drive.Color2.green()>41) //Needs to be at least >76
//        {
//            level = 2;
//            telemetry.addData("Level:",level);
//            telemetry.addData("Color:",drive.Color.green());
//            telemetry.addData("Color2:",drive.Color2.green());
//            telemetry.update();
//        }
//        else
//        {
//            level = 1;
//            telemetry.addData("Level:",level);
//            telemetry.addData("Color:",drive.Color.green());
//            telemetry.addData("Color2:",drive.Color2.green());
//            telemetry.update();
//        }
        if(drive.Color.red()>60) //Needs to be at least >93
        {
            level = 3;
            telemetry.addData("Level:", level);
            telemetry.addData("Color:", drive.Color.green());
            telemetry.addData("Color2:", drive.Color2.green());
            telemetry.update();
        }
        else if (drive.Color2.red()>45) //Needs to be at least >76
        {
            level = 2;
            telemetry.addData("Level:",level);
            telemetry.addData("Color:",drive.Color.green());
            telemetry.addData("Color2:",drive.Color2.green());
            telemetry.update();
        }
        else
        {
            level = 1;
            telemetry.addData("Level:",level);
            telemetry.addData("Color:",drive.Color.green());
            telemetry.addData("Color2:",drive.Color2.green());
            telemetry.update();
        }
        sleep(1000); //Scan
        drive.followTrajectorySequence(t3);
        boolean done = false;
        switch (level)
        {
            case 1: {
                telemetry.addData("Level:", 1);
                telemetry.update();
                drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.Lift.setDirection(DcMotor.Direction.FORWARD);
                drive.Indexer.setPosition(BoxLiftingPosition);
                drive.Lift.setTargetPosition(480); //560 is normal but a little too much
                drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.Lift.setPower(.4);
                lifttime.reset();
                while (drive.Lift.isBusy() && opModeIsActive()&&lifttime.seconds()<1.5) {
                    telemetry.addData("Height:", drive.Lift.getCurrentPosition());
                    telemetry.update();
                }
                drive.Lift.setPower(0);
//        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Starting:", "Dump & Lower");
                telemetry.update();
                //Dump and lower:
                //position while dumped
                drive.Indexer.setPosition(BoxDumpingPosition);
                sleep(600);
                drive.Indexer.setPosition(BoxStartingPosition);
                //drive.Indexer.setPosition(BoxLiftingPosition);
                //drive.Lift.setDirection(DcMotor.Direction.FORWARD);
                //int Liftgoal = 0;
                //drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        drive.Lift.setPower(.4);
//        while (drive.Lift.getCurrentPosition()>Liftgoal && !opModeIsActive()) {
//            telemetry.addData("Level:",level);
//            telemetry.addData("Lowering, rn at Height:", drive.Lift.getCurrentPosition());
//            telemetry.update();
//        }
                drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive.Lift.setPower(-.4);
                //drive.Lift.setTargetPosition(-560);
                //drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                while (drive.Lift.isBusy() && opModeIsActive()) {
//                }
//                    drive.Lift.setPower(0);
                lifttime.reset();
                while(!drive.Touchy.isPressed()&&lifttime.seconds()<1.5)
                {
                    drive.Lift.setPower(-.4);
                }
                drive.Lift.setPower(0);
            }
            break;
            case 2: {
                telemetry.addData("Level:", 2);
                telemetry.update();
                drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.Lift.setDirection(DcMotor.Direction.FORWARD);
                drive.Indexer.setPosition(BoxLiftingPosition);
                drive.Lift.setTargetPosition(1120);
                drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.Lift.setPower(.4);
                lifttime.reset();
                while (drive.Lift.isBusy() && opModeIsActive()&&lifttime.seconds()<2) {
                    telemetry.addData("Height:", drive.Lift.getCurrentPosition());
                    telemetry.update();
                }
                drive.Lift.setPower(0);
//        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Starting:", "Dump & Lower");
                telemetry.update();
                //Dump and lower:
                //position while dumped
                drive.Indexer.setPosition(BoxDumpingPosition);
                sleep(600);
                drive.Indexer.setPosition(BoxStartingPosition);
                //drive.Indexer.setPosition(BoxLiftingPosition);
                //drive.Lift.setDirection(DcMotor.Direction.FORWARD);
                //int Liftgoal = 0;
                //drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        drive.Lift.setPower(.4);
//        while (drive.Lift.getCurrentPosition()>Liftgoal && !opModeIsActive()) {
//            telemetry.addData("Level:",level);
//            telemetry.addData("Lowering, rn at Height:", drive.Lift.getCurrentPosition());
//            telemetry.update();
//        }
                drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive.Lift.setPower(-.4);
                //drive.Lift.setTargetPosition(-560);
                //drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                while (drive.Lift.isBusy() && opModeIsActive()) {
//                }
//                    drive.Lift.setPower(0);
                lifttime.reset();
                while(!drive.Touchy.isPressed()&&lifttime.seconds()<2)
                {
                    drive.Lift.setPower(-.4);
                }
                drive.Lift.setPower(0);
            }
            break;
            case 3: {
                telemetry.addData("Level:", 3);
                telemetry.update();
                drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.Lift.setDirection(DcMotor.Direction.FORWARD);
                drive.Indexer.setPosition(BoxLiftingPosition);
                drive.Lift.setTargetPosition(1680); //560 is normal but a little too much
                drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.Lift.setPower(.4);
                lifttime.reset();
                while (drive.Lift.isBusy() && opModeIsActive()&&lifttime.seconds()<3) {
                    telemetry.addData("Height:", drive.Lift.getCurrentPosition());
                    telemetry.update();
                }
                drive.Lift.setPower(0);
//        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Starting:", "Dump & Lower");
                telemetry.update();
                //Dump and lower:
                //position while dumped
                drive.Indexer.setPosition(BoxDumpingPosition);
                sleep(600);
                drive.Indexer.setPosition(BoxStartingPosition);
                //drive.Indexer.setPosition(BoxLiftingPosition);
                //drive.Lift.setDirection(DcMotor.Direction.FORWARD);
                //int Liftgoal = 0;
                //drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        drive.Lift.setPower(.4);
//        while (drive.Lift.getCurrentPosition()>Liftgoal && !opModeIsActive()) {
//            telemetry.addData("Level:",level);
//            telemetry.addData("Lowering, rn at Height:", drive.Lift.getCurrentPosition());
//            telemetry.update();
//        }
                drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive.Lift.setPower(-.4);
                //drive.Lift.setTargetPosition(-560);
                //drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                while (drive.Lift.isBusy() && opModeIsActive()) {
//                }
//                    drive.Lift.setPower(0);
                lifttime.reset();
                while(!drive.Touchy.isPressed()&&lifttime.seconds()<3)
                {
                    drive.Lift.setPower(-.4);
                }
                drive.Lift.setPower(0);
                drive.followTrajectorySequence(special);
                //sleep(10000);

                drive.Color.enableLed(false);
                drive.Color2.enableLed(false);
                done = true;
            }
            break;
        }
        if(!done){
            sleep(2000); //Place
            drive.followTrajectorySequence(t4);
            drive.Color.enableLed(false);
            drive.Color2.enableLed(false);
            //sleep(10000);
            //drive.followTrajectorySequence(trajSeq1);
        }

        Pose2d poseEstimate2 = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate2.getX());
        telemetry.addData("finalY", poseEstimate2.getY());
        telemetry.addData("finalHeading", poseEstimate2.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;

    }
}