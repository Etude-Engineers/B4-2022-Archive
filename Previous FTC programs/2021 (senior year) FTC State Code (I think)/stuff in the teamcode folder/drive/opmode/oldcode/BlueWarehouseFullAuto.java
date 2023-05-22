package org.firstinspires.ftc.teamcode.drive.opmode.oldcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Disabled
@Autonomous(group = "drive")
public class BlueWarehouseFullAuto extends LinearOpMode {
//Simple Plan #?


    final double BoxStartingPosition = 0.55;
    final double BoxLiftingPosition = 0.7;
    final double BoxDumpingPosition = 1;
    int level = 1;
    private ElapsedTime lifttime = new ElapsedTime();
    private ElapsedTime Intaketime = new ElapsedTime();




    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        //We will have Sensors on the back:
        //Pose2d startingPosition = new Pose2d(-40.875,61.75, Math.toRadians(90)); //If we Scan first

        //Right near 2&3
        //Line up the inside of the right side of the drive train bar with the seam closest to spinner
        Pose2d startingPosition = new Pose2d(6.625, 58.75, Math.toRadians(90)); //If we Spin First, off by 1.5in
        drive.setPoseEstimate(startingPosition);

        drive.Color.enableLed(true);
        drive.Color2.enableLed(true);


        //Goes right to Scan 2&3
        TrajectorySequence t1 = drive.trajectorySequenceBuilder(startingPosition)
                //.back(7.5)
                .lineToConstantHeading(new Vector2d(6.625, 45.675)) //y was like 4 inches too little, x was like an inch too little
                .build();

        //Placing Block:
        TrajectorySequence t2 = drive.trajectorySequenceBuilder(t1.end())
                //.strafeLeft(17.125)
                .lineToConstantHeading(new Vector2d(-11.75, 39.25)) //seems like 1.5 inch off
                .build();

//        //Go to Warehouse to collect next block
        TrajectorySequence t3 = drive.trajectorySequenceBuilder(t2.end())
                .splineToSplineHeading(new Pose2d(21.5,64,Math.toRadians(0)),Math.toRadians(0)) //At Barrier, off by 1.5 in in y
//                .addDisplacementMarker(() -> {
//                    drive.Intake.setPower(1); })
        //At the Blocks:
                .splineToConstantHeading(new Vector2d(42,64),Math.toRadians(0))
                .build();

        //Get's completely outside of barrier:
        Pose2d poseEstimate = drive.getPoseEstimate();
//        TrajectorySequence t5 = drive.trajectorySequenceBuilder(poseEstimate)
//                .splineToConstantHeading(new Vector2d(0,64.825),Math.toRadians(0))
//                .build();




        waitForStart();

        if (isStopRequested()) return;
        drive.followTrajectorySequence(t1);
        if(drive.Color.green()>110) //Needs to be at least >93
        {
            level = 3;
            telemetry.addData("Level:",level);
            telemetry.addData("Color:",drive.Color.green());
            telemetry.addData("Color2:",drive.Color2.green());
            telemetry.update();
        }
        else if (drive.Color2.green()>65) //Needs to be at least >76
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
            telemetry.update();
        }
        //sleep(100); //Scan
        drive.followTrajectorySequence(t2); // Goes to Hub
//        switch (level)
//        {
//            case 1: {
//                telemetry.addData("Level:", 1);
//                telemetry.update();
//                drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                drive.Lift.setDirection(DcMotor.Direction.FORWARD);
//                drive.Indexer.setPosition(BoxLiftingPosition);
//                drive.Lift.setTargetPosition(560); //560 is normal but a little too much
//                drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                drive.Lift.setPower(.4);
//                lifttime.reset();
//                while (drive.Lift.isBusy() && opModeIsActive()&&lifttime.seconds()<1.5) {
//                    telemetry.addData("Height:", drive.Lift.getCurrentPosition());
//                    telemetry.update();
//                }
//                drive.Lift.setPower(0);
////        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                telemetry.addData("Starting:", "Dump & Lower");
//                telemetry.update();
//                //Dump and lower:
//                //position while dumped
//                drive.Indexer.setPosition(BoxDumpingPosition);
//                sleep(600);
//                drive.Indexer.setPosition(BoxStartingPosition);
//                //drive.Indexer.setPosition(BoxLiftingPosition);
//                //drive.Lift.setDirection(DcMotor.Direction.FORWARD);
//                //int Liftgoal = 0;
//                //drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        drive.Lift.setPower(.4);
////        while (drive.Lift.getCurrentPosition()>Liftgoal && !opModeIsActive()) {
////            telemetry.addData("Level:",level);
////            telemetry.addData("Lowering, rn at Height:", drive.Lift.getCurrentPosition());
////            telemetry.update();
////        }
//                drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                drive.Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                drive.Lift.setPower(-.4);
//                //drive.Lift.setTargetPosition(-560);
//                //drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                while (drive.Lift.isBusy() && opModeIsActive()) {
////                }
////                    drive.Lift.setPower(0);
//                lifttime.reset();
//                while(!drive.Touchy.isPressed()&&lifttime.seconds()<1.5)
//                {
//                    drive.Lift.setPower(-.4);
//                    //drive.setMotorPowers(.8,.8,.8,.8);
//                }
//                drive.Lift.setPower(0);
//            }
//            break;
//            case 2: {
//                telemetry.addData("Level:", 2);
//                telemetry.update();
//                drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                drive.Lift.setDirection(DcMotor.Direction.FORWARD);
//                drive.Indexer.setPosition(BoxLiftingPosition);
//                drive.Lift.setTargetPosition(1120);
//                drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                drive.Lift.setPower(.4);
//                lifttime.reset();
//                while (drive.Lift.isBusy() && opModeIsActive()&&lifttime.seconds()<2) {
//                    telemetry.addData("Height:", drive.Lift.getCurrentPosition());
//                    telemetry.update();
//                }
//                drive.Lift.setPower(0);
////        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                telemetry.addData("Starting:", "Dump & Lower");
//                telemetry.update();
//                //Dump and lower:
//                //position while dumped
//                drive.Indexer.setPosition(BoxDumpingPosition);
//                sleep(600);
//                drive.Indexer.setPosition(BoxStartingPosition);
//                //drive.Indexer.setPosition(BoxLiftingPosition);
//                //drive.Lift.setDirection(DcMotor.Direction.FORWARD);
//                //int Liftgoal = 0;
//                //drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        drive.Lift.setPower(.4);
////        while (drive.Lift.getCurrentPosition()>Liftgoal && !opModeIsActive()) {
////            telemetry.addData("Level:",level);
////            telemetry.addData("Lowering, rn at Height:", drive.Lift.getCurrentPosition());
////            telemetry.update();
////        }
//                drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                drive.Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                drive.Lift.setPower(-.4);
//                //drive.Lift.setTargetPosition(-560);
//                //drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                while (drive.Lift.isBusy() && opModeIsActive()) {
////                }
////                    drive.Lift.setPower(0);
//                lifttime.reset();
//                while(!drive.Touchy.isPressed()&&lifttime.seconds()<2)
//                {
//                    drive.Lift.setPower(-.4);
//                    //drive.setMotorPowers(.8,.8,.8,.8);
//                }
//                drive.Lift.setPower(0);
//            }
//            break;
//            case 3: {
//                telemetry.addData("Level:", 3);
//                telemetry.update();
//                drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                drive.Lift.setDirection(DcMotor.Direction.FORWARD);
//                drive.Indexer.setPosition(BoxLiftingPosition);
//                drive.Lift.setTargetPosition(1680); //560 is normal but a little too much
//                drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                drive.Lift.setPower(.4);
//                lifttime.reset();
//                while (drive.Lift.isBusy() && opModeIsActive()&&lifttime.seconds()<3) {
//                    telemetry.addData("Height:", drive.Lift.getCurrentPosition());
//                    telemetry.update();
//                }
//                drive.Lift.setPower(0);
////        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                telemetry.addData("Starting:", "Dump & Lower");
//                telemetry.update();
//                //Dump and lower:
//                //position while dumped
//                drive.Indexer.setPosition(BoxDumpingPosition);
//                sleep(600);
//                drive.Indexer.setPosition(BoxStartingPosition);
//                //drive.Indexer.setPosition(BoxLiftingPosition);
//                //drive.Lift.setDirection(DcMotor.Direction.FORWARD);
//                //int Liftgoal = 0;
//                //drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        drive.Lift.setPower(.4);
////        while (drive.Lift.getCurrentPosition()>Liftgoal && !opModeIsActive()) {
////            telemetry.addData("Level:",level);
////            telemetry.addData("Lowering, rn at Height:", drive.Lift.getCurrentPosition());
////            telemetry.update();
////        }
//                drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                drive.Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                drive.Lift.setPower(-.4);
//                //drive.Lift.setTargetPosition(-560);
//                //drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                while (drive.Lift.isBusy() && opModeIsActive()) {
////                }
////                    drive.Lift.setPower(0);
//                lifttime.reset();
//                while(!drive.Touchy.isPressed()&&lifttime.seconds()<3)
//                {
//                    drive.Lift.setPower(-.4);
//                    //drive.setMotorPowers(.8,.8,.8,.8);
//                }
//                drive.Lift.setPower(0);
//            }
//            break;
//        }
        //sleep(2000); //Place
        drive.followTrajectorySequence(t3); //Going to pickup

        //Comment out later maybe:
        drive.Indexer.setPosition(BoxStartingPosition);

        while (drive.Distance.getDistance(DistanceUnit.INCH)>1.5)
        {
            drive.Intake.setPower(1);
            drive.setMotorPowers(0.2,0.2,.2,.2);
        }
        telemetry.addData("Got it!", drive.Distance.getDistance(DistanceUnit.INCH));
        telemetry.update();

        drive.setMotorPowers(0,0,0,0);
        drive.Intake.setPower(-1);
        drive.Indexer.setPosition(BoxLiftingPosition);

        //Figures out where we are:
        poseEstimate = drive.getPoseEstimate();
        TrajectorySequence t5 = drive.trajectorySequenceBuilder(poseEstimate)
                .strafeLeft(2)
                .lineToConstantHeading(new Vector2d(5,65))
                .addDisplacementMarker(()->{
            drive.Intake.setPower(0);
        })
                //.splineToConstantHeading(new Vector2d(5,65),Math.toRadians(0))
                .strafeRight(6)
                .build();
        drive.followTrajectorySequence(t5); //Get out of barrier
        TrajectorySequence t6 = drive.trajectorySequenceBuilder(t5.end())
                .splineToLinearHeading(new Pose2d(-12.75,38.75, Math.toRadians(90)),Math.toRadians(90)) //Y seemed off by like 1.5, maybe cause of hitting the wall so much
                .build();
        drive.followTrajectorySequence(t6); //Goes to Hub
        //Intake drops it off
//        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        drive.Lift.setDirection(DcMotor.Direction.FORWARD);
//        drive.Indexer.setPosition(BoxLiftingPosition);
//        drive.Lift.setTargetPosition(1680); //560 is normal but a little too much
//        drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        drive.Lift.setPower(.4);
//        lifttime.reset();
//        while (drive.Lift.isBusy() && opModeIsActive()&&lifttime.seconds()<3) {
//            telemetry.addData("Height:", drive.Lift.getCurrentPosition());
//            telemetry.update();
//        }
//        drive.Lift.setPower(0);
////        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        telemetry.addData("Starting:", "Dump & Lower");
//        telemetry.update();
//        //Dump and lower:
//        //position while dumped
//        drive.Indexer.setPosition(BoxDumpingPosition);
//        sleep(600);
//        drive.Indexer.setPosition(BoxStartingPosition);
//        //drive.Indexer.setPosition(BoxLiftingPosition);
//        //drive.Lift.setDirection(DcMotor.Direction.FORWARD);
//        //int Liftgoal = 0;
//        //drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        drive.Lift.setPower(.4);
////        while (drive.Lift.getCurrentPosition()>Liftgoal && !opModeIsActive()) {
////            telemetry.addData("Level:",level);
////            telemetry.addData("Lowering, rn at Height:", drive.Lift.getCurrentPosition());
////            telemetry.update();
////        }
//        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        drive.Lift.setPower(-.4);
//        //drive.Lift.setTargetPosition(-560);
//        //drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                while (drive.Lift.isBusy() && opModeIsActive()) {
////                }
////                    drive.Lift.setPower(0);
//        lifttime.reset();
        drive.followTrajectorySequence(t3); //Going to Park/Pickup
        //Comment out later maybe:
        drive.Indexer.setPosition(BoxStartingPosition);

        while (drive.Distance.getDistance(DistanceUnit.INCH)>1.5)
        {
            drive.Intake.setPower(1);
            drive.setMotorPowers(0.2,0.2,.2,.2);
        }
        telemetry.addData("Got it!", drive.Distance.getDistance(DistanceUnit.INCH));
        drive.setMotorPowers(0,0,0,0);
        drive.Intake.setPower(-1);
        drive.Indexer.setPosition(BoxLiftingPosition);

        //Figures out where we are:
        poseEstimate = drive.getPoseEstimate();
        TrajectorySequence t7 = drive.trajectorySequenceBuilder(poseEstimate)
                .strafeLeft(2)
                .splineToConstantHeading(new Vector2d(5,65),Math.toRadians(0))
                .addDisplacementMarker(()->{
                    drive.Intake.setPower(0);
                })
                .strafeRight(6)
                .build();
        drive.followTrajectorySequence(t7); //Get out of barrier
        TrajectorySequence t8 = drive.trajectorySequenceBuilder(t7.end())
                .splineToLinearHeading(new Pose2d(-12.75,38.25,Math.toRadians(90)),Math.toRadians(90))
                .build();
        drive.followTrajectorySequence(t8); //Goes to Hub
        drive.followTrajectorySequence(t3); //Going to Park/Pickup




        Pose2d poseEstimate2 = drive.getPoseEstimate();


        while (!isStopRequested() && opModeIsActive()) ;

    }
}