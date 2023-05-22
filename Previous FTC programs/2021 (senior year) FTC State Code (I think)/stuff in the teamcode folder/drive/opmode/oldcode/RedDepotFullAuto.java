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
public class RedDepotFullAuto extends LinearOpMode {
//Simple Plan #?


    final double BoxStartingPosition = 0.55;
    final double BoxLiftingPosition = 0.7;
    final double BoxDumpingPosition = 1;
    int level = 1;
    private ElapsedTime lifttime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        //We will have Sensors on the back:
        //Pose2d startingPosition = new Pose2d(-40.875,61.75, Math.toRadians(90)); //If we Scan first

        //Line up the right side with the seam closest to spinner
        Pose2d startingPosition = new Pose2d(-30.3, -61.75, Math.toRadians(270)); //If we Spin First, off by 1.5in
        drive.setPoseEstimate(startingPosition);
        drive.Color.enableLed(true);
        drive.Color2.enableLed(true);

        //Spinning the Duck
        Trajectory t3 = drive.trajectoryBuilder(startingPosition)
                .splineTo(new Vector2d(-61.75, -65.825),Math.toRadians(180)) //Goes like an inch less then it should, I think becuase it's so close
                .build();

        //Scanning Position
        //2 Parts, first moves away from wall, 2nd moves straight to the position
        TrajectorySequence t1 = drive.trajectorySequenceBuilder(startingPosition)
                //.back(7.5)
                //Goes to 2,3 still, close to Shipping then blue
                .lineToConstantHeading(new Vector2d(-30.3, -48.675)) //y was like 2 inches too little, x is off by a little
                .build();


        //Placing Block:
//        Trajectory t3 = drive.trajectoryBuilder(t2.end())
//                .lineToConstantHeading(new Vector2d(-10.75, 41.75)) //x was like an inch too little
//                .build()
        TrajectorySequence t2 = drive.trajectorySequenceBuilder(t1.end())
                .strafeLeft(16)
                .lineToConstantHeading(new Vector2d(-10.25, -41.75))
                .build();
//        Trajectory t3 = drive.trajectoryBuilder(t2.end())
//                .splineTo(new Vector2d(-11.75, 41.75), Math.toRadians(90))
//                .build();

        //Parking in Depot
        TrajectorySequence t4 = drive.trajectorySequenceBuilder(t3.end())
                //.forward(5)
                .lineToConstantHeading(new Vector2d(-60, -37.75)) //Center from x seems to be 2 inches off, Center for y seems to be 6 inches from what should have been center
                .build();
//        TrajectorySequence special = drive.trajectorySequenceBuilder(t3.end())
//                .lineToConstantHeading(new Vector2d(-25, -55)) //x was like an inch too little
////                .splineTo(new Vector2d(-60, 35.25), Math.toRadians(180))
//                .lineToConstantHeading(new Vector2d(-60, -35.25)) //x was like an inch too little
//                .build();

        waitForStart();

        if (isStopRequested()) return;
//        drive.followTrajectory(t1);
//        drive.Spinner.setPower(-0.3);
//        telemetry.addData("Color:",drive.Color.green());
//        telemetry.update();
//        sleep(4000); //Spin
//        drive.Spinner.setPower(0);
        sleep(7000);
        drive.followTrajectorySequence(t1);
//        if(drive.Color.green()>110) //Needs to be at least >110 I think
//        {
//            level = 3;
//            telemetry.addData("Level:",level);
//            telemetry.addData("Color:",drive.Color.green());
//            telemetry.addData("Color2:",drive.Color2.green());
//            telemetry.update();
//        }
//        else if (drive.Color2.green()>50) //Needs to be at least >79
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
        sleep(250); //Scan
        drive.followTrajectorySequence(t2);
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
                drive.Lift.setTargetPosition(560); //560 is normal but a little too much
                drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.Lift.setPower(1);
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
                sleep(400);
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
                drive.Lift.setPower(-1);
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
                //                drive.followTrajectorySequence(special);
//                //sleep(10000);
//
//                drive.Color.enableLed(false);
//                drive.Color2.enableLed(false);
//                done = true;
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
                drive.Lift.setPower(1);
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
                sleep(400);
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
                drive.Lift.setPower(-1);
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
                drive.Lift.setPower(1);
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
                sleep(400);
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
                drive.Lift.setPower(-1);
                //drive.Lift.setTargetPosition(-560);
                //drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                while (drive.Lift.isBusy() && opModeIsActive()) {
//                }
//                    drive.Lift.setPower(0);
                lifttime.reset();
                while(!drive.Touchy.isPressed()&&lifttime.seconds()<3)
                {
                    drive.Lift.setPower(-.8);
                }
                drive.Lift.setPower(0);
            }
            break;
        }
        if(!done){
            drive.followTrajectory(t3);
            drive.Spinner.setPower(-.3);
            sleep(4000); //Spin
            drive.Spinner.setPower(0);
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