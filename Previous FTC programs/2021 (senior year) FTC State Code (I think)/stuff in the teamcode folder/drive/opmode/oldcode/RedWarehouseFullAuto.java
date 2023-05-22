package org.firstinspires.ftc.teamcode.drive.opmode.oldcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Config
@Autonomous(group = "drive")
public class RedWarehouseFullAuto extends LinearOpMode {
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
        Pose2d startingPosition = new Pose2d(6.625, -58.75, Math.toRadians(270)); //If we Spin First, off by 1.5in
        drive.setPoseEstimate(startingPosition);

        drive.Color.enableLed(true);
        drive.Color2.enableLed(true);


        //Goes right to Scan 2&3
        TrajectorySequence t1 = drive.trajectorySequenceBuilder(startingPosition)
                //.back(7.5)
                .lineToConstantHeading(new Vector2d(6.625, -45.675)) //y was like 4 inches too little, x was like an inch too little
                .build();

        //Placing Block:
        TrajectorySequence t2 = drive.trajectorySequenceBuilder(t1.end())
                .strafeRight(17.125)
                .lineToConstantHeading(new Vector2d(-11.75, -38.25)) //
                .build();

//        //Go to Warehouse to collect next block
        TrajectorySequence t3 = drive.trajectorySequenceBuilder(t2.end())
                .splineToSplineHeading(new Pose2d(13.5,-63,Math.toRadians(0)),Math.toRadians(0)) //At Barrier, off by .5 in y, 3 in x
//                .addDisplacementMarker(() -> {
//                    drive.Intake.setPower(1); })
                .splineToConstantHeading(new Vector2d(45,-62),Math.toRadians(0))
                .build();

        //Go to Allaince Hub to drop off block
        TrajectorySequence t4 = drive.trajectorySequenceBuilder(t3.end())
                .splineToConstantHeading(new Vector2d(27,-68.825),Math.toRadians(0)) //Seems to be a bit off but this may make it hit the wall, although maybe thats ok
//                .addDisplacementMarker(() -> {
//                    drive.Intake.setPower(0);
//                })
//                .splineToSplineHeading(new Pose2d(-12.75,39.75),Math.toRadians(270))
                .build();
        TrajectorySequence t5 = drive.trajectorySequenceBuilder(t4.end())
                .splineToConstantHeading(new Vector2d(12.5,-68.825),Math.toRadians(0))
                .build();
        TrajectorySequence t6 = drive.trajectorySequenceBuilder(t5.end())
                .splineToSplineHeading(new Pose2d(21.5,-63,Math.toRadians(270)),Math.toRadians(0)) //At Barrier, off by 1.5 in in y
                .build();



        waitForStart();

        if (isStopRequested()) return;
        drive.followTrajectorySequence(t1);
        if(drive.Color.green()>110) //Needs to be at least >93
        {
            level = 2;
            telemetry.addData("Level:",level);
            telemetry.addData("Color:",drive.Color.green());
            telemetry.addData("Color2:",drive.Color2.green());
            telemetry.update();
        }
        else if (drive.Color2.green()>65) //Needs to be at least >76
        {
            level = 1;
            telemetry.addData("Level:",level);
            telemetry.addData("Color:",drive.Color.green());
            telemetry.addData("Color2:",drive.Color2.green());
            telemetry.update();
        }
        else
        {
            level = 3;
            telemetry.addData("Level:",level);
            telemetry.update();
        }
        sleep(250); //Scan
        drive.followTrajectorySequence(t2);
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
                    //drive.setMotorPowers(.8,.8,.8,.8);
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
                    //drive.setMotorPowers(.8,.8,.8,.8);
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
                    //drive.setMotorPowers(.8,.8,.8,.8);
                }
                drive.Lift.setPower(0);
            }
            break;
        }
        //sleep(2000); //Place
        drive.followTrajectorySequence(t3); //Going to pickup
//        while(Intaketime.seconds()<1&&drive.Distance.getDistance(DistanceUnit.INCH)>1.5) //Intake
//        {
//            drive.Intake.setPower(1);
//        }
//        while (drive.Distance.getDistance(DistanceUnit.INCH)>1.5)
//        {
//            drive.setMotorPowers(0.125,0.125,.125,.125);
//        }
//            telemetry.addData("Got it!", drive.Distance.getDistance(DistanceUnit.INCH));
//        drive.Indexer.setPosition(BoxLiftingPosition);
//        drive.Intake.setPower(-1);
//        drive.followTrajectorySequence(t4);
//        drive.followTrajectorySequence(t5);
//        drive.Intake.setPower(0);
//        drive.followTrajectorySequence(t6);
//        //Intake drops it off
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
        //drive.followTrajectorySequence(t3); //Going to Park/Pickup





        Pose2d poseEstimate2 = drive.getPoseEstimate();


        while (!isStopRequested() && opModeIsActive()) ;

    }
}