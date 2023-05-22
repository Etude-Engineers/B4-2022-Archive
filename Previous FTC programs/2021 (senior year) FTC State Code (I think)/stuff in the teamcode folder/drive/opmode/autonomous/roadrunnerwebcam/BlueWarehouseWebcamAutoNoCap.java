package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.roadrunnerwebcam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.autonomous.webcam.FreightFrenzyDeterminationPipeline_1_0;
import org.firstinspires.ftc.teamcode.drive.opmode.teleop.LiftPID;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@Autonomous(group = "drive")
public class BlueWarehouseWebcamAutoNoCap extends LinearOpMode {

    LiftPID CustomLiftPID = new  LiftPID(.001,0,0,0,0);

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    //Webcam and detection initialization:
    OpenCvWebcam webcam;
    FreightFrenzyDeterminationPipeline_1_0 pipeline;
    FreightFrenzyDeterminationPipeline_1_0.FreightPosition snapshotAnalysis = FreightFrenzyDeterminationPipeline_1_0.FreightPosition.LEFT; // default
    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take

    enum State {
        DriveToHub,   //Drives to the Hub and raises the lift to the right level and raises the capstone arm
        DumpLowerGoToWarehouse,   //Go to dumping position, start moving back to warehouse and then slightly after starting to move back
        //put the bucket back to lifting positon and lower the lift
//        IntakeUntilTouchSensor, //Drive forward slowly and run the intake until we sense we have a block in our intake
        BacktoHub,   //Go back to hub, run the intake until the block is at the distance sensor, then lift up to the 3rd level while intaking backwards.
        Park,
        //After BacktoHub do DumpLowerGoToWarehouse again
        IDLE            // Our bot will enter the IDLE state when done
    }


    double LiftSpeed = 1;

    static double HowfarIntakeGoes = 55;
    static double AroundBarriersSidePos = 66.75;
    static double HowfarIntoWarehouseRobotGoes = 40;
    static double WarehouseIncrement = 4;
    static double Angle = 2;
    static double incrment = 0.375+.375; //was 0.4375, .5
    static double HubIncrement = -0.375;

    //TODO: Tune Max Velocity and Acceleration
    //This file specfic:
    public static double maxVel = 50;
    public static double maxAccel = 50;
    public static double maxAngularVel = 180;
    public static double Divider = 16.6;

    //Box variables
    final double BoxStartingPosition = 0.55;
    final double BoxLiftingPosition = 0.7;
    final double BoxDumpingPosition = 1;

    //Variable for the Distance Sensor:
    double distanceSensitivity = 1.95;


    //Total Elapsed time:
    private ElapsedTime TotalTime = new ElapsedTime();
    //Potentially useful for checking how much time our cycles are taking
    private ElapsedTime FirstBlock = new ElapsedTime();
    double TookFirstLong = 30;
    private ElapsedTime SecondBlock = new ElapsedTime();
    double TookSecondLong = 30;
    private ElapsedTime ThirdBlock = new ElapsedTime();
    double TookThirdLong = 30;
    double AtBarriers = 17.5;


    //State of the intake:
    Boolean donewithIntake = false;
    Boolean hasABlock = false;
    Boolean DonewithLift = false;

    //State of Lift:
    Boolean donewithLift = false;
    //State of CapstoneArm:
    Boolean donewithCapstoneArm = false;

//    Boolean hasDetected = false;

    //How many blocks we are aiming for:
    double GoalBlocksToGet = 4;
    //If this much time is left dont score anymore:
    double WhenTooLittleTimeIsLeft = 24;

    //Timers for the lift, detecting, Intake or Capstone if needed
    private ElapsedTime Detecttime = new ElapsedTime();
    private ElapsedTime Lifttime = new ElapsedTime();
    private ElapsedTime Intaketime = new ElapsedTime();
    private ElapsedTime Capstonetime = new ElapsedTime();


    //Default to IDLE and resets variables
    State currentState = State.IDLE;
    public double AmountofBlocksGotten = 0;
    int level = 3;
    int LiftPos = 1680;


    //In the middle of the pieces
    Pose2d poseEstimate;
    Pose2d startingPosition = new Pose2d(12.5, 61.75, Math.toRadians(90)); //If we Spin First, off by 1.5in

    @Override
    public void runOpMode() throws InterruptedException {


        //Detection Setup:
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new FreightFrenzyDeterminationPipeline_1_0();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });



        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set inital pose
        drive.setPoseEstimate(startingPosition);
        TrajectorySequence DriveToHub = drive.trajectorySequenceBuilder(startingPosition) //Goes to Hub:
                .setReversed(true)
                .lineTo(new Vector2d(-11,40.5))
                .build();
        TrajectorySequence ToWarehouse;
//        = drive.trajectorySequenceBuilder(poseEstimate)
//                .setReversed(false)
//                .splineTo(new Vector2d(25,65),Math.toRadians(0))
//                .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes,65),Math.toRadians(0))
//                .setConstraints(drive.getVelocityConstraint(maxVel/Divider, Math.toRadians(maxAngularVel/Divider), 15.53),drive.getAccelerationConstraint(maxAccel))
//                .splineToConstantHeading(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))
//                .build();
        TrajectorySequence BacktoHub;
        TrajectorySequence Park;

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
            switch (pipeline.getAnalysis()) {
                case LEFT: {
                    level = 1;
                    LiftPos =480;
                    drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    break;
                }
                case CENTER: {
                    level = 2;
                    LiftPos = 1020;
                    drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    break;
                }
                case RIGHT: {
                    level = 3;
                    LiftPos = 1680;
                    drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    break;
                }
            }

        }

        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */
        snapshotAnalysis = pipeline.getAnalysis();
        switch (snapshotAnalysis) {
            case LEFT: {
                level = 1;
                LiftPos = 480;
                drive.Lift.setDirection(DcMotor.Direction.FORWARD);

//                drive.Lift.setTargetPosition(480);
//                drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                break;
            }
            case CENTER: {
                level = 2;
                LiftPos = 1020;
                drive.Lift.setDirection(DcMotor.Direction.FORWARD);
//                drive.Lift.setTargetPosition(1120);
//                drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                break;
            }
            case RIGHT: {
                level = 3;
                LiftPos = 1680;
                drive.Lift.setDirection(DcMotor.Direction.FORWARD);
//                drive.Lift.setTargetPosition(1680);
//                drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                break;
            }
        }

        /*
         * Show that snapshot on the telemetry
         */

            telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
            telemetry.addData("level:", level);
            telemetry.update();



        if (isStopRequested()) return;


        //Set the current state to DrivetoGamepeice, our first step, Then have it follow that trajectory
        //Because of async it lets the program continue
        TotalTime.reset();
        FirstBlock.reset();
        drive.Indexer.setPosition(BoxLiftingPosition);
        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.Lift.setDirection(DcMotor.Direction.FORWARD);

        drive.followTrajectorySequenceAsync(DriveToHub);
        currentState = State.DriveToHub; //Goes to blocks
        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case DriveToHub:
//                    drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    telemetry.addData("State: ", "DriveToHub");
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished



                    if (drive.Lift.getCurrentPosition()<480&&level==1) { //Runs the lift to the right hight
                        telemetry.addData("Level:", 1);
                        telemetry.addData("Height:", drive.Lift.getCurrentPosition());
                        telemetry.update();
                        drive.Lift.setPower(1);
                    }
                    else if(level==1)
                    {
                        drive.Lift.setPower(0);
                    }

                    if (drive.Lift.getCurrentPosition()<1020&&level==2) { //Runs the lift to the right hight
                        telemetry.addData("Level:", 1);
                        telemetry.addData("Height:", drive.Lift.getCurrentPosition());
                        telemetry.update();
                        drive.Lift.setPower(1);
                    }
                    else if(level==2)
                    {
                        drive.Lift.setPower(0);
                    }

                    if (drive.Lift.getCurrentPosition()<1680&&level==3) { //Runs the lift to the right hight
                        telemetry.addData("Level:", 1);
                        telemetry.addData("Height:", drive.Lift.getCurrentPosition());
                        telemetry.update();
                        drive.Lift.setPower(1);
                    }
                    else if (level==3)
                    {
                        drive.Lift.setPower(0);
                    }



//                    if (drive.Lift.getCurrentPosition()<480&&level==1&&!donewithLift) { //Runs the lift to the right hight
//                        telemetry.addData("Level:", 1);
//                        telemetry.addData("Height:", drive.Lift.getCurrentPosition());
//                        telemetry.update();
//                        drive.Lift.setPower(1);
//                    }
//                    if (drive.Lift.getCurrentPosition()>=480&&level==1&&!donewithLift) { //Runs the lift to the right hight
//                        donewithLift = true;
//                    }
//                    if (drive.Lift.getCurrentPosition()<480&&level==1&&donewithLift) { //Runs the lift to the right hight
//                        drive.Lift.setPower(0); //Stopped Lift
//                        if(CustomLiftPID.Calculate(drive.Lift.getCurrentPosition(),480,0.02)>0) {
//                            drive.Lift.setPower(CustomLiftPID.Calculate(drive.Lift.getCurrentPosition(), 480, 0.02));
//                        }
////                    Lifttimer.reset();
//                        telemetry.addData("Lift target Height:", 480);
//                        telemetry.addData("Lift Current Height:", drive.Lift.getCurrentPosition());
//                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(drive.Lift.getCurrentPosition(), 480, 0.02));
//                        telemetry.update();
//                    }
//                    else if (drive.Lift.getCurrentPosition()<480&&level==1&&!donewithLift) { //Runs the lift to the right hight
//                        telemetry.addData("Level:", 1);
//                        telemetry.addData("Height:", drive.Lift.getCurrentPosition());
//                        telemetry.update();
//                        drive.Lift.setPower(1);
//                    }
//
//
//
//                    if (drive.Lift.getCurrentPosition()<1120&&level==2&&!donewithLift) { //Runs the lift to the right hight
//                        telemetry.addData("Level:", 1);
//                        telemetry.addData("Height:", drive.Lift.getCurrentPosition());
//                        telemetry.update();
//                        drive.Lift.setPower(1);
//                    }
//                    if (drive.Lift.getCurrentPosition()>=1120&&level==2&&!donewithLift) { //Runs the lift to the right hight
//                        donewithLift = true;
//                    }
//                    if (drive.Lift.getCurrentPosition()<1120&&level==2&&donewithLift) { //Runs the lift to the right hight
//                        drive.Lift.setPower(0); //Stopped Lift
//                        if(CustomLiftPID.Calculate(drive.Lift.getCurrentPosition(),1120,0.02)>0) {
//                            drive.Lift.setPower(CustomLiftPID.Calculate(drive.Lift.getCurrentPosition(), 1120, 0.02));
//                        }
////                    Lifttimer.reset();
//                        telemetry.addData("Lift target Height:", 1120);
//                        telemetry.addData("Lift Current Height:", drive.Lift.getCurrentPosition());
//                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(drive.Lift.getCurrentPosition(), 1120, 0.02));
//                        telemetry.update();
//                    }
//                    else if (drive.Lift.getCurrentPosition()<1120&&level==2&&!donewithLift) { //Runs the lift to the right hight
//                        telemetry.addData("Level:", 1);
//                        telemetry.addData("Height:", drive.Lift.getCurrentPosition());
//                        telemetry.update();
//                        drive.Lift.setPower(1);
//                    }
//
//
//                    if (drive.Lift.getCurrentPosition()<1680&&level==3&&!donewithLift) { //Runs the lift to the right hight
//                        telemetry.addData("Level:", 1);
//                        telemetry.addData("Height:", drive.Lift.getCurrentPosition());
//                        telemetry.update();
//                        drive.Lift.setPower(1);
//                    }
//                    if (drive.Lift.getCurrentPosition()>=1680&&level==3&&!donewithLift) { //Runs the lift to the right hight
//                        donewithLift = true;
//                    }
//                    if (drive.Lift.getCurrentPosition()<1680&&level==3&&donewithLift) { //Runs the lift to the right hight
//                        drive.Lift.setPower(0); //Stopped Lift
//                        if(CustomLiftPID.Calculate(drive.Lift.getCurrentPosition(),1680,0.02)>0) {
//                            drive.Lift.setPower(CustomLiftPID.Calculate(drive.Lift.getCurrentPosition(), 1680, 0.02));
//                        }
////                    Lifttimer.reset();
//                        telemetry.addData("Lift target Height:", 1680);
//                        telemetry.addData("Lift Current Height:", drive.Lift.getCurrentPosition());
//                        telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(drive.Lift.getCurrentPosition(), 1680, 0.02));
//                        telemetry.update();
//                    }
//                    else if (drive.Lift.getCurrentPosition()<1680&&level==3&&!donewithLift) { //Runs the lift to the right hight
//                        telemetry.addData("Level:", 1);
//                        telemetry.addData("Height:", drive.Lift.getCurrentPosition());
//                        telemetry.update();
//                        drive.Lift.setPower(1);
//                    }

                    //Might want to make sure the Lift isnt busy here:
                    if (!drive.isBusy()) { //When finished (the lift should be done too) dump then go back to warehouse

                        drive.Lift.setPower(0);
                        drive.Indexer.setPosition(BoxDumpingPosition); //Dumps
                        if(AmountofBlocksGotten == 1)
                        {
                            sleep(375);
                        }
                            else if(AmountofBlocksGotten == 2)
                            {
                                sleep(375);
                            }
                                else if(AmountofBlocksGotten == 3)
                                {
                                    sleep(500);
                                }
                                else
                        {
                            sleep(500);
                        }

//                        drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

                        //Prepares the lift for lowering:
//                        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Lifttime.reset();

                        //Record we scored a block:
                        AmountofBlocksGotten = 1;


                        TookFirstLong = FirstBlock.seconds();
                        telemetry.addData("AutoTime:", TotalTime.seconds());
                        telemetry.addData("First block took:", TookFirstLong);
                        telemetry.update();
                        SecondBlock.reset();
//                        telemetry.addData("Trajectory: ", "DumpLowerGoToWarehouse");

                        //Sets the drive to go to the hub regardless where it is right now
//                        poseEstimate = drive.getPoseEstimate();
//                        DumpLowerGoToWarehouse = drive.trajectorySequenceBuilder(poseEstimate)
////                                .splineToLinearHeading(new Pose2d(8.5,65, Math.toRadians(2)),Math.toRadians(2)) //At Barrier, off by 1.5 in in y
////                                .splineToConstantHeading(new Vector2d(44,65),Math.toRadians(0))
//                                .splineTo(new Vector2d(12,65),Math.toRadians(0)) //At Barrier, off by 1.5 in in y
//                                .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes,65.5),Math.toRadians(0))
//                                .setConstraints(drive.getVelocityConstraint(maxVel/Divider, maxAngularVel/Divider, TRACK_WIDTH),drive.getAccelerationConstraint(maxAccel))
//                                .splineTo(new Vector2d(HowfarIntakeGoes,66),Math.toRadians(0))
//                                .build();

                        if (!drive.isBusy() && AmountofBlocksGotten < GoalBlocksToGet) { //if we
                            hasABlock = false;
                            donewithIntake = false;
                            poseEstimate = drive.getPoseEstimate();
                            drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
                            telemetry.addData("trajectory, initial, Ammount of Blocks Gotten:",1);
                            telemetry.update();
                            ToWarehouse = drive.trajectorySequenceBuilder(DriveToHub.end())
                                .setReversed(false)
                                    .splineTo(new Vector2d(AtBarriers, 65), Math.toRadians(2)) //At Barrier, off by 1.5 in in y
//                                    .splineTo(new Vector2d(5,60),Math.toRadians(40))
                                    .splineToConstantHeading(new Vector2d(HowfarIntoWarehouseRobotGoes,65),Math.toRadians(0))
//                                            .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes,65.5),Math.toRadians(0))
                                //angle can be not 0, but has to use the spline to constant heading then
//                                    .setVelConstraint(drive.getVelocityConstraint(maxVel/Divider, Math.toRadians(maxAngularVel), 15.53))
//                                    .setReversed(false)
                                    //If usees an angle other then 0 use this one instead:
                                .splineToConstantHeading(new Vector2d(HowfarIntakeGoes,66),Math.toRadians(0)
                                        , getVelocityConstraint(maxVel/Divider, Math.toRadians(180/Divider), 15.53),getAccelerationConstraint(maxAccel)
                                       )//At Barrier, off by 1.5 in in y  )
//                                .splineTo(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))
                                    .build();
                            //Prep Lift:
                            drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            drive.Lift.setDirection(DcMotor.Direction.FORWARD);
                            drive.Lift.setPower(0);
                            Lifttime.reset();
                            drive.followTrajectorySequenceAsync(ToWarehouse); //Going back to warehouse
                            currentState = State.DumpLowerGoToWarehouse;
                        }

//                        if (!drive.isBusy() && AmountofBlocksGotten >= (GoalBlocksToGet)) { //If we have done enough stop
//                            hasABlock = false;
//                            donewithIntake = false;
//                            poseEstimate = drive.getPoseEstimate();
//                            Park = drive.trajectorySequenceBuilder(poseEstimate)
////                                .splineToLinearHeading(new Pose2d(8.5,65, Math.toRadians(2)),Math.toRadians(2)) //At Barrier, off by 1.5 in in y
////                                .splineToConstantHeading(new Vector2d(44,65),Math.toRadians(0))
//                                    .splineTo(new Vector2d(12,65),Math.toRadians(0)) //At Barrier, off by 1.5 in in y
//                                    .splineTo(new Vector2d(44,65.5),Math.toRadians(0))
//                                    .build();
//                            drive.followTrajectorySequence(Park); //Going back to warehouse
//                            Lifttime.reset();
//                            currentState = State.Park;
//                        }

                    }
                    break;
                case DumpLowerGoToWarehouse: //Goes to the warehouse, Lowers the lift
                    telemetry.addData("State: ", "DumpLowerGoToWarehouse");


                    //waits half a second before lowering lift:
                    if (!drive.Touchy.isPressed()&&Lifttime.seconds()<3&&Lifttime.seconds()>1) //If Lift isnt down, keeps doing down:
                    {
//                        drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                        drive.Indexer.setPosition(BoxStartingPosition); //resets
                        telemetry.addData("Lift Going Down:", drive.Lift.getCurrentPosition());
                        telemetry.addData("Lift Speed:", -1);
                        telemetry.addData("AutoTime:", TotalTime.seconds());
                        telemetry.addData("First block took:", TookFirstLong);
                        telemetry.update();
                        drive.Lift.setPower(-1);
                    }
                    else
                    {
                        if(Lifttime.seconds()<=1) //Lift Waits before lowering
                        {
                            drive.Lift.setPower(0);
//                            drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
                            telemetry.addData("Lift Waiting:", Lifttime.seconds());
                        }
                        else//When down stops:
                        {
//                            drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                            telemetry.addData("Lift Waiting:", Lifttime.seconds());
                            telemetry.addData("Lift Down:", drive.Lift.getCurrentPosition());
                            telemetry.addData("AutoTime:", TotalTime.seconds());
                            telemetry.addData("First block took:", TookFirstLong);
                            telemetry.update();
                            drive.Lift.setPower(0);
                        }
                    }


//                    if((drive.IntakeColor.alpha()>100)||drive.Distance.getDistance(DistanceUnit.INCH)<distanceSensitivity)
//                    {
//                        Pose2d poseEstimate = drive.getPoseEstimate();
//                        TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(poseEstimate)
////                                .strafeLeft(2)
//                                .setReversed(true)
//                                .splineToLinearHeading(new Pose2d(5, 65, Math.toRadians(0)), Math.toRadians(180)) //Gets just outside of warehouse
////                                .splineTo(new Vector2d(-11.75, 44.25), Math.toRadians(-90))
//                                .splineTo(new Vector2d(-11.75, 39.25), Math.toRadians(-90))
//                                .build();
//                        drive.followTrajectorySequenceAsync(trajectory4);
//                        currentState = State.TRAJECTORY_4;
//                    }

                    //Done with Intake
                    if (drive.Distance.getDistance(DistanceUnit.INCH) < distanceSensitivity &&Lifttime.seconds()>1) {
                        drive.Indexer.setPosition(BoxLiftingPosition);
                        drive.Intake.setPower(-1);

                        donewithIntake = true;
                        drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

                        telemetry.addData("A block is in the box!", "Running the intake backwards now");
                        telemetry.update();
                    }

                    //Block is in intake?
                    else if (drive.IntakeColor.alpha()>120 &&Lifttime.seconds()>1) {
                        drive.Indexer.setPosition(BoxStartingPosition);
                        drive.Intake.setPower(1);

                        hasABlock = true;
                        drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);

                        telemetry.addData("A block touched my sensor!", "running the intake still");
                        telemetry.addData("Distance is:", drive.Distance.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    }


                    else if (Lifttime.seconds()>1) {
                        telemetry.addData("Distance is:", drive.Distance.getDistance(DistanceUnit.INCH));
                        telemetry.addData("Color is?", drive.IntakeColor.alpha());
                        telemetry.update();

                        //Runs Intake if nothing is in it yet:
                        drive.Indexer.setPosition(BoxStartingPosition);
                        drive.Intake.setPower(1);
                    }


                    //Moves onto next, If we still have time to keep going:
                    if ((donewithIntake || hasABlock) && AmountofBlocksGotten < GoalBlocksToGet && TotalTime.seconds() < WhenTooLittleTimeIsLeft) {
                        Pose2d poseEstimate = drive.getPoseEstimate();

                        if(AmountofBlocksGotten == 1) {
                            //The trajectory that goes back to the hub:
                            BacktoHub = drive.trajectorySequenceBuilder(poseEstimate)
//                                .strafeLeft(2)
                                    .setConstraints(drive.getVelocityConstraint(maxVel, Math.toRadians(maxAngularVel), 15.53), drive.getAccelerationConstraint(maxAccel))
                                    .setReversed(true)
                                    .splineTo(new Vector2d(12, 65 + AmountofBlocksGotten * incrment+0.375), Math.toRadians(180 - 2)) //Gets just outside of warehouse
                                    .splineTo(new Vector2d(-11, 41 + AmountofBlocksGotten * HubIncrement), Math.toRadians(275))
                                    .build();
                        }
                        if(AmountofBlocksGotten == 2) {
                            //The trajectory that goes back to the hub:
                            BacktoHub = drive.trajectorySequenceBuilder(poseEstimate)
//                                .strafeLeft(2)
                                    .setConstraints(drive.getVelocityConstraint(maxVel, Math.toRadians(maxAngularVel), 15.53), drive.getAccelerationConstraint(maxAccel))
                                    .setReversed(true)
                                    .splineTo(new Vector2d(12, 65 + AmountofBlocksGotten * incrment+0.75), Math.toRadians(180 - 2)) //Gets just outside of warehouse
                                    .splineTo(new Vector2d(-11, 41 + AmountofBlocksGotten * HubIncrement), Math.toRadians(275))
                                    .build();
                        }
                        if(AmountofBlocksGotten == 3) {
                            //The trajectory that goes back to the hub:
                            BacktoHub = drive.trajectorySequenceBuilder(poseEstimate)
//                                .strafeLeft(2)
                                    .setConstraints(drive.getVelocityConstraint(maxVel, Math.toRadians(maxAngularVel), 15.53), drive.getAccelerationConstraint(maxAccel))
                                    .setReversed(true)
                                    .splineTo(new Vector2d(12, 65 + AmountofBlocksGotten * incrment+0.75), Math.toRadians(180 - 2)) //Gets just outside of warehouse
                                    .splineTo(new Vector2d(-11, 41 + AmountofBlocksGotten * HubIncrement), Math.toRadians(275))
                                    .build();
                        }
                        else
                        {
                            BacktoHub = drive.trajectorySequenceBuilder(poseEstimate)
//                                .strafeLeft(2)
                                    .setConstraints(drive.getVelocityConstraint(maxVel, Math.toRadians(maxAngularVel), 15.53), drive.getAccelerationConstraint(maxAccel))
                                    .setReversed(true)
                                    .splineTo(new Vector2d(12, 65 + AmountofBlocksGotten * incrment), Math.toRadians(180 - 2)) //Gets just outside of warehouse
                                    .splineTo(new Vector2d(-11, 41 + AmountofBlocksGotten * HubIncrement), Math.toRadians(275))
                                    .build();
                        }


                        //More telemetry:
                        telemetry.addData("AutoTime:", TotalTime.seconds());
                        if (donewithIntake) {
                            telemetry.addData("A block is is in the box!", "running the intake backwards still");
                            drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        }
                        else if (hasABlock) {
                            telemetry.addData("A block touched the touch sensor!", "Going to score ot now");
                            drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                        }
                        telemetry.addData("Distance is:", drive.Distance.getDistance(DistanceUnit.INCH));
                        telemetry.update();


                        //Follow next trajectory: (back to the hub)
                        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        drive.followTrajectorySequenceAsync(BacktoHub);
                        currentState = State.BacktoHub;
                    }


                    //If has gotten enough blocks just stop:
                    if ((donewithIntake || hasABlock) && (AmountofBlocksGotten >= GoalBlocksToGet || TotalTime.seconds() >= WhenTooLittleTimeIsLeft)) {

                        telemetry.addData("Parking, got this many blocks:", AmountofBlocksGotten);
                        if (TotalTime.seconds() >= WhenTooLittleTimeIsLeft) {
                            telemetry.addData("Too little time left!", TotalTime.seconds());
                        }
                        if (drive.Distance.getDistance(DistanceUnit.INCH) < distanceSensitivity) {
                            drive.Indexer.setPosition(BoxLiftingPosition);
                            drive.Intake.setPower(-1);

                            donewithIntake = true;
                            drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

                            telemetry.addData("A block is in the box!", "Running the intake backwards now");
                            telemetry.addData("Distance is:", drive.Distance.getDistance(DistanceUnit.INCH));
                            telemetry.update();
                        }
                        else{
                            drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                            drive.Intake.setPower(0);
                        }
                        currentState = State.IDLE;
                    }


                    break;
                case BacktoHub: //go back to hub, run the intake until the block is at the distance sensor, then runs the lift,
                    telemetry.addData("We've got this many blocks!", AmountofBlocksGotten);
                    telemetry.addData("State: ", "BacktoHub");


                    //Intakes until distance sensor:
                     if (drive.Distance.getDistance(DistanceUnit.INCH) < distanceSensitivity) {
                        drive.Indexer.setPosition(BoxLiftingPosition);
                        drive.Intake.setPower(-1);
                        donewithIntake = true;
                        drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        telemetry.addData("A block is in the box!", "Running the intake backwards now");
                        telemetry.update();
                    } else {
                        drive.Indexer.setPosition(BoxStartingPosition);
                        drive.Intake.setPower(1);
                    }


                    if(donewithIntake) {

                        if (drive.Lift.getCurrentPosition() < 1680 + AmountofBlocksGotten * 40 && !donewithLift) { //Runs the lift to the right hight
                            telemetry.addData("Level:", 1);
                            telemetry.addData("Height:", drive.Lift.getCurrentPosition());
                            telemetry.update();
                            drive.Lift.setPower(1);
                        }
                        if (drive.Lift.getCurrentPosition() >= 1680 && !donewithLift) { //Runs the lift to the right hight
                            donewithLift = true;
                        }
                        if (drive.Lift.getCurrentPosition() < 1680 && donewithLift) { //Runs the lift to the right hight
                            drive.Lift.setPower(0); //Stopped Lift
                            if (CustomLiftPID.Calculate(drive.Lift.getCurrentPosition(), 1680, 0.02) > 0) {
                                drive.Lift.setPower(CustomLiftPID.Calculate(drive.Lift.getCurrentPosition(), 1680, 0.02));
                            }
//                    Lifttimer.reset();
                            telemetry.addData("Lift target Height:", 1680);
                            telemetry.addData("Lift Current Height:", drive.Lift.getCurrentPosition());
                            telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(drive.Lift.getCurrentPosition(), 1680, 0.02));
                            telemetry.update();
                        } else if (drive.Lift.getCurrentPosition() < 1680 && !donewithLift) { //Runs the lift to the right hight
                            telemetry.addData("Level:", 1);
                            telemetry.addData("Height:", drive.Lift.getCurrentPosition());
                            telemetry.update();
                            drive.Lift.setPower(1);
                        }

                    }
                    else
                    {
                        drive.Lift.setPower(0);
                    }



//                     //When Intake is done runs the Lift to the top:
//                    if (drive.Lift.getCurrentPosition() < (1680 + AmountofBlocksGotten * 40) && donewithIntake) {
//
//                        drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//                        telemetry.addData("Lifting!","");
////                        telemetry.addData("Was pressure plate touched?", hasABlock);
////                        telemetry.addData("Was the distance sensor close?", donewithIntake);
//                        drive.Lift.setPower(LiftSpeed);
//                        telemetry.update();
//                    } else {
//                        drive.Lift.setPower(0);
//                    }

                    //When everything is done it dumps
                    if (!drive.isBusy()) {
                        telemetry.addData("Got here!","");
                        drive.Indexer.setPosition(BoxDumpingPosition); //Dumps
                        drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
                        sleep(500);
                        Lifttime.reset();
                        drive.Intake.setPower(0);
                        AmountofBlocksGotten++;

                        drive.Lift.setDirection(DcMotorSimple.Direction.FORWARD);
//                        drive.Lift.setPower(-LiftSpeed);
                        hasABlock = false;
                        donewithIntake = false;
                        //gets position
//                        poseEstimate = drive.getPoseEstimate();
                        telemetry.addData("We got this many blocks!", AmountofBlocksGotten);
                        telemetry.update();
                        if(AmountofBlocksGotten<GoalBlocksToGet) {
                        if(AmountofBlocksGotten==1) //just did scan:
                        {
                            //Goes further the second time we pick up a block
                            TrajectorySequence DumpLowerGoToWarehouse = drive.trajectorySequenceBuilder(poseEstimate)
                                    .setReversed(false)
                                    .splineTo(new Vector2d(25,AroundBarriersSidePos+incrment),Math.toRadians(2)) //At Barrier, off by 1.5 in in y
                                    .splineToConstantHeading(new Vector2d(HowfarIntoWarehouseRobotGoes,AroundBarriersSidePos+incrment),Math.toRadians(0))
                                    .setVelConstraint(drive.getVelocityConstraint(maxVel/Divider, Math.toRadians(maxAngularVel/Divider), 15.53))
//                                    .setConstraints(drive.getVelocityConstraint(maxVel/Divider, Math.toRadians(maxAngularVel/Divider), 15.53),drive.getAccelerationConstraint(maxAccel))
                                    .splineToConstantHeading(new Vector2d(HowfarIntakeGoes,AroundBarriersSidePos+1.375),Math.toRadians(0))
//                                            , getVelocityConstraint(maxVel/Divider, Math.toRadians(180), DriveConstants.TRACK_WIDTH),getAccelerationConstraint(maxAccel))
//                                .splineTo(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))
                                    .build();
                            drive.followTrajectorySequenceAsync(DumpLowerGoToWarehouse);
                            currentState = State.DumpLowerGoToWarehouse;
                        }
                            if (AmountofBlocksGotten == 2) //just did first after scan:
                            {
                                drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                                telemetry.addData("trajectory, Ammount of Blocks Gotten:",2);
                                telemetry.update();
                                //Goes further the second time we pick up a block
                                ToWarehouse = drive.trajectorySequenceBuilder(DriveToHub.end())
                                    .setReversed(false)
                                        .splineTo(new Vector2d(AtBarriers, AroundBarriersSidePos+incrment*2), Math.toRadians(2)) //At Barrier, off by 1.5 in in y
                                        .splineToConstantHeading(new Vector2d(HowfarIntoWarehouseRobotGoes + WarehouseIncrement, AroundBarriersSidePos+incrment*2), Math.toRadians(0))
//                                        .setConstraints(drive.getVelocityConstraint(maxVel / Divider, Math.toRadians(maxAngularVel / Divider), 15.53), drive.getAccelerationConstraint(maxAccel))
                                        .splineToConstantHeading(new Vector2d(HowfarIntakeGoes + WarehouseIncrement, AroundBarriersSidePos+1+incrment*2), Math.toRadians(0),
                                                getVelocityConstraint(maxVel/Divider, Math.toRadians(180/Divider), 15.53),getAccelerationConstraint(maxAccel))
//                                .splineTo(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))
                                        .build();
                                drive.followTrajectorySequenceAsync(ToWarehouse);
                                currentState = State.DumpLowerGoToWarehouse;
                            } else if (AmountofBlocksGotten == 3) //If this was second after scan:
                            {
                                drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
                                telemetry.addData("trajectory, Ammount of Blocks Gotten:",3);
                                telemetry.update();
                                //Goes further the second time we pick up a block
                                ToWarehouse = drive.trajectorySequenceBuilder(DriveToHub.end())
                                    .setReversed(false)
                                        .splineTo(new Vector2d(AtBarriers, AroundBarriersSidePos+incrment*2), Math.toRadians(2)) //At Barrier, off by 1.5 in in y
                                        .splineToConstantHeading(new Vector2d(HowfarIntoWarehouseRobotGoes + 2*WarehouseIncrement, AroundBarriersSidePos+incrment*2), Math.toRadians(0))

//                                        .setConstraints(drive.getVelocityConstraint(maxVel / Divider, Math.toRadians(maxAngularVel / Divider), 15.53), drive.getAccelerationConstraint(maxAccel))
                                        .splineToConstantHeading(new Vector2d(HowfarIntakeGoes + 2*WarehouseIncrement-2, AroundBarriersSidePos+1+incrment*2), Math.toRadians(0),
                                                getVelocityConstraint(maxVel/Divider, Math.toRadians(180/Divider), 15.53),getAccelerationConstraint(maxAccel))
//                                .splineTo(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))
                                        .build();
                                drive.followTrajectorySequenceAsync(ToWarehouse);
                                currentState = State.DumpLowerGoToWarehouse;
                            }
//                            else if (AmountofBlocksGotten == 4) //If this was third after scan:
//                            {
//                                telemetry.addData("trajectory, Ammount of Blocks Gotten:",4);
//                                telemetry.update();
//                                drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
//                                //Goes further the second time we pick up a block
//                                ToWarehouse = drive.trajectorySequenceBuilder(DriveToHub.end())
//                                    .setReversed(false)
//                                        .splineTo(new Vector2d(AtBarriers, AroundBarriersSidePos+incrment*3), Math.toRadians(0)) //At Barrier, off by 1.5 in in y
//                                        .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes + 10, AroundBarriersSidePos+incrment*3), Math.toRadians(0))
//
////                                        .setConstraints(drive.getVelocityConstraint(maxVel / Divider, Math.toRadians(maxAngularVel / Divider), 15.53), drive.getAccelerationConstraint(maxAccel))
//                                        .splineTo
//                                                (new Vector2d(HowfarIntakeGoes + 10, AroundBarriersSidePos+1+incrment*3), Math.toRadians(0),
//                                                        getVelocityConstraint(maxVel/Divider, Math.toRadians(180/Divider), 15.53),getAccelerationConstraint(maxAccel))
////                                  .splineTo(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))
//                                        .build();
//                                drive.followTrajectorySequenceAsync(ToWarehouse);
//                                currentState = State.DumpLowerGoToWarehouse;
//                            }
                            else { //parking:
                                drive.Intake.setPower(0);
                                Park = drive.trajectorySequenceBuilder(DriveToHub.end())
                                        .setReversed(false)
                                        .splineTo(new Vector2d(AtBarriers,AroundBarriersSidePos+incrment*4),Math.toRadians(2)) //At Barrier, off by 1.5 in in y
                                        .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes+3*WarehouseIncrement,AroundBarriersSidePos+1+incrment*4),Math.toRadians(0))
                                        .build();
                                //Amount of blocks gotten = 1 follow normal trajectory
                                drive.followTrajectorySequenceAsync(Park);
                                currentState = State.Park;
                            }
                        }
                        else { //parking:
                            drive.Intake.setPower(0);
                            telemetry.addData("trajectory, parking","");
                            telemetry.update();
                            Park = drive.trajectorySequenceBuilder(poseEstimate)
                                    .setReversed(false)
                                    .splineTo(new Vector2d(AtBarriers,AroundBarriersSidePos+incrment*4),Math.toRadians(2)) //At Barrier, off by 1.5 in in y
                                    .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes+3*WarehouseIncrement,AroundBarriersSidePos+incrment*4),Math.toRadians(0))
                                    .build();
                            //Amount of blocks gotten = 1 follow normal trajectory
                            drive.followTrajectorySequenceAsync(Park);
                            currentState = State.Park;
                        }
                    }
                    break;
                case Park:
                    telemetry.addData("Parking!", "Driving there right now");
                    if(drive.isBusy())
                    {
                        currentState = State.IDLE;
                    }
                    break;

                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
//            lift.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            //PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.update();
        }
    }

    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
//    class Lift {
//        public Lift(HardwareMap hardwareMap) {
//            // Beep boop this is the the constructor for the lift
//            // Assume this sets up the lift hardware
//        }
//
//        public void update() {
//            // Beep boop this is the lift update function
//            // Assume this runs some PID controller for the lift
//        }
//    }
}