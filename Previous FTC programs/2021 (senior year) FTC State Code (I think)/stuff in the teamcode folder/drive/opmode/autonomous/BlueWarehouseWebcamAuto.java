package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.autonomous.webcam.FreightFrenzyDeterminationPipeline_1_0;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

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
@Disabled
@Autonomous(group = "drive")
public class BlueWarehouseWebcamAuto extends LinearOpMode {

    //Webcam and detection initialization:
    OpenCvWebcam webcam;
    FreightFrenzyDeterminationPipeline_1_0 pipeline;
    FreightFrenzyDeterminationPipeline_1_0.FreightPosition snapshotAnalysis = FreightFrenzyDeterminationPipeline_1_0.FreightPosition.LEFT; // default
    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    Trajectory DrivetoGamepeice;
    TrajectorySequence DriveToHub;
    TrajectorySequence DumpLowerGoToWarehouse;
    TrajectorySequence BacktoHub;
    TrajectorySequence Park;
    enum State {
        DrivetoGamepeice,   //First, go to pick up the game peice and either lower the capstone while driving, or use the next state to
        LowerCapstoneArm,   //Might be done in first state
        DriveToHub,   //Drives to the Hub and raises the lift to the right level and raises the capstone arm

        //Up to here has been tuned with meep meep
        DumpLowerGoToWarehouse,   //Go to dumping position, start moving back to warehouse and then slightly after starting to move back
        //but the bucket back to lifting positon and lower the lift
//        IntakeUntilTouchSensor, //Drive forward slowly and run the intake until we sense we have a block in our intake
        BacktoHub,   //Go back to hub, run the intake until the block is at the distance sensor, then lift up to the 3rd level while intaking backwards.
        Park,
        //After BacktoHub do DumpLowerGoToWarehouse again
        IDLE            // Our bot will enter the IDLE state when done
    }


    double LiftSpeed = 1;

    static double HowfarIntakeGoes = 55;
    static double HowfarIntoWarehouseRobotGoes = 41.5;

    //TODO: Tune Max Velocity and Acceleration
    //This file specfic:
    public static double maxVel = 50;
    public static double maxAccel = 50;
    public static double maxAngularVel = 180;
    public static double Divider = 3.5;

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

    //State of the intake:
    Boolean donewithIntake = false;
    Boolean hasABlock = false;

    //State of Lift:
    Boolean donewithLift = false;
    //State of CapstoneArm:
    Boolean donewithCapstoneArm = false;

//    Boolean hasDetected = false;

    //How many blocks we are aiming for:
    double GoalBlocksToGet = 5;
    //If this much time is left dont score anymore:
    double WhenTooLittleTimeIsLeft = 30;

    //Timers for the lift, detecting, Intake or Capstone if needed
    private ElapsedTime Detecttime = new ElapsedTime();
    private ElapsedTime Lifttime = new ElapsedTime();
    private ElapsedTime Intaketime = new ElapsedTime();
    private ElapsedTime Capstonetime = new ElapsedTime();


    //Default to IDLE and resets variables
    State currentState = State.IDLE;
    public double AmountofBlocksGotten = 0;
    int level = 3;


    //In the middle of the pieces
    Pose2d poseEstimate;
    Pose2d startingPosition = new Pose2d(12.5, 58.75, Math.toRadians(90)); //If we Spin First, off by 1.5in

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

        // Set inital pose
        drive.setPoseEstimate(startingPosition);

//        // Let's define our trajectories
//        Trajectory DrivetoGamepeice = drive.trajectoryBuilder(startingPosition) //Goes to Scan:
//                .lineToConstantHeading(new Vector2d(6.625, 45.675)) //y was like 4 inches too little, x was like an inch too little
//                .build();
//
//        // Second trajectory
//        // Ensure that we call trajectory1.end() as the start for this one
//        TrajectorySequence DriveToHub = drive.trajectorySequenceBuilder(DrivetoGamepeice.end()) //Goes to Hub:
//                .strafeLeft(12)
//                .splineToConstantHeading(new Vector2d(-12.75, 39.25),Math.toRadians(90)) //seems like 1.5 inch off
//                .build();
//
//        // Third trajectory
//        //Going to Warehouse
//        // Ensure that we call trajectory2.end() as the start for this one
//        TrajectorySequence DumpLowerGoToWarehouse = drive.trajectorySequenceBuilder(DriveToHub.end())
//                .splineToLinearHeading(new Pose2d(8.5,65, Math.toRadians(2)),Math.toRadians(2)) //At Barrier, off by 1.5 in in y
//                .splineToConstantHeading(new Vector2d(44,65),Math.toRadians(0))
//                .build();
        // Fourth trajectory is going back to the hub but to have the correct starting pos we have to make it below




        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
            switch (pipeline.getAnalysis()) {
                case LEFT: {
                    level = 1;
                    drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    break;
                }
                case CENTER: {
                    level = 2;
                    drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    break;
                }
                case RIGHT: {
                    level = 3;
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


        /*
         * Show that snapshot on the telemetry
         */
        switch (snapshotAnalysis) {
            case LEFT: {
                level = 1;
                drive.CapstoneArm.setPosition(0.2);
                TrajectorySequence DrivetoGamepeice;
                DrivetoGamepeice = drive.trajectorySequenceBuilder(startingPosition) //Goes to pick up capstone:
                        .setReversed(true)
                        .addTemporalMarker(0.5, () -> {
                            drive.CapstoneHand.setPosition(.375);
                        })
                        .splineToLinearHeading(new Pose2d(9, 45.175,Math.toRadians(315)), Math.toRadians(215))
//                        .splineTo(new Vector2d(9, 45.175),Math.toRadians(315))
                        .build();
                drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                drive.CapstoneArm.setPosition(0.6);
                drive.followTrajectorySequenceAsync(DrivetoGamepeice);
                break;
            }

            case CENTER: {
                level = 2;
                TrajectorySequence DrivetoGamepeice;
                DrivetoGamepeice = drive.trajectorySequenceBuilder(startingPosition) //Goes to pick up capstone:\
                        .lineToConstantHeading(new Vector2d(12.5, 45.675))
                        .build();
                drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                drive.CapstoneArm.setPosition(0.6);
                drive.followTrajectorySequenceAsync(DrivetoGamepeice);
                break;
            }
            case RIGHT: {
                level = 3;
                TrajectorySequence DrivetoGamepeice;
                DrivetoGamepeice = drive.trajectorySequenceBuilder(startingPosition) //Goes to pick up capstone:
                        .lineToConstantHeading(new Vector2d(3.5, 45.675))
                        .build();
                drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                drive.CapstoneArm.setPosition(0.6);
                drive.followTrajectorySequenceAsync(DrivetoGamepeice);
                break;
            }
        }
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


        currentState = State.DrivetoGamepeice; //Goes to blocks
        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case DrivetoGamepeice:

                    telemetry.addData("State: ", "DrivetoGamepeice");
                    telemetry.addData("level:", level);
                    telemetry.update();
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Because of the async follow function we do the whats not in if(!drive.isBusy()) constantly
                    if (!drive.isBusy()) {//Once it gets to blocks detects the block and then starts the lift

                        //Reset the lift and get it ready to go to the right level:
                        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        drive.Lift.setDirection(DcMotor.Direction.FORWARD);
                        drive.Indexer.setPosition(BoxLiftingPosition);

                        if(level==1)
                        {
                            drive.Lift.setTargetPosition(480);

                        }
                        else {
                            drive.Lift.setTargetPosition(560 * level);
                        }

                        drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Lifttime.reset();

                        //Sets the drive to go to the hub regardless where it is right now
//                        poseEstimate = drive.getPoseEstimate();
//                        DriveToHub = drive.trajectorySequenceBuilder(poseEstimate) //Goes to Hub:
//                                .splineToLinearHeading(new Pose2d(-3, 40,Math.toRadians(60)),Math.toRadians(210)) //seems like 1.5 inch off
//                                .build();

                        currentState = State.LowerCapstoneArm;
//                        currentState = State.DriveToHub;
//                        drive.followTrajectorySequenceAsync(DriveToHub);
                    }
           break;
                case LowerCapstoneArm:
                    //grabs
                    sleep(200);
                    drive.CapstoneHand.setPosition(0.75);
                    sleep(200);
                    drive.CapstoneArm.setPosition(0);
                    poseEstimate = drive.getPoseEstimate();
                    DriveToHub = drive.trajectorySequenceBuilder(poseEstimate) //Goes to Hub:
                            .splineToLinearHeading(new Pose2d(-9, 41,Math.toRadians(60)),Math.toRadians(210)) //seems like 1.5 inch off
                            .build();
                    currentState = State.DriveToHub;
                    drive.followTrajectorySequenceAsync(DriveToHub);
                    break;
                case DriveToHub:
                    telemetry.addData("State: ", "DriveToHub");
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (drive.Lift.isBusy() && Lifttime.seconds() < level + 0.5) { //Runs the lift to the right hight
                        telemetry.addData("Height:", drive.Lift.getCurrentPosition());
                        telemetry.update();
                        drive.Lift.setPower(1);
                    }

                    //Might want to make sure the Lift isnt busy here:
                    if (!drive.isBusy()) { //When finished (the lift should be done too) dump then go back to warehouse

                        drive.Indexer.setPosition(BoxDumpingPosition); //Dumps
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
                            DumpLowerGoToWarehouse = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(false)
//                                    .splineTo(new Vector2d(5,60),Math.toRadians(40))
                                    .splineTo(new Vector2d(25,65),Math.toRadians(0)) //At Barrier, off by 1.5 in in y                                .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes,65.5),Math.toRadians(0))
                                //angle can be not 0, but has to use the spline to constant heading then
                                .setConstraints(drive.getVelocityConstraint(maxVel/Divider, Math.toRadians(maxAngularVel/Divider), 15.53),drive.getAccelerationConstraint(maxAccel))
                                //If usees an angle other then 0 use this one instead:
                                .splineToConstantHeading(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))
//                                .splineTo(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))
                                    .build();
                            drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);

                            //Prep Lift:
                            drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            drive.Lift.setDirection(DcMotor.Direction.FORWARD);
                            drive.Lift.setPower(0);
                            Lifttime.reset();

                            drive.followTrajectorySequenceAsync(DumpLowerGoToWarehouse); //Going back to warehouse
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
                    if (!drive.Touchy.isPressed()&&Lifttime.seconds()>1) //If Lift isnt down, keeps doing down:
                    {
                        drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                        drive.Indexer.setPosition(BoxStartingPosition); //resets
                        telemetry.addData("Lift Going Down:", drive.Lift.getCurrentPosition());
                        telemetry.addData("Lift Speed:", -1);
                        telemetry.addData("AutoTime:", TotalTime.seconds());
                        telemetry.addData("First block took:", TookFirstLong);
                        telemetry.update();
                        drive.Lift.setPower(-1);
                    } else
                    {
                        if(Lifttime.seconds()<=1) //Lift Waits before lowering
                        {
                            drive.Lift.setPower(0);
                            drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
                            telemetry.addData("Lift Waiting:", Lifttime.seconds());
                        }
                        else//When down stops:
                        {
                            drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
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
                    else if (drive.IntakeColor.alpha()>100 &&Lifttime.seconds()>1) {
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

                        //The trajectory that goes back to the hub:
                        BacktoHub = drive.trajectorySequenceBuilder(poseEstimate)
//                                .strafeLeft(2)
                                .setConstraints(drive.getVelocityConstraint(maxVel, Math.toRadians(maxAngularVel), 15.53),drive.getAccelerationConstraint(maxAccel))
                                .setReversed(true)
                                .splineTo(new Vector2d(12,65),Math.toRadians(180)) //Gets just outside of warehouse
                                .splineTo(new Vector2d(-3, 40),Math.toRadians(240))
                                .build();


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
                    telemetry.addData("State: ", "BacktoHub");
                    telemetry.addData("Is the distance sensor close?", drive.Distance.getDistance(DistanceUnit.INCH) < distanceSensitivity);

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

                     //When Intake is done runs the Lift to the top:
                    if (drive.Lift.getCurrentPosition() < (1680 + AmountofBlocksGotten * 50) && donewithIntake) {
                        telemetry.addData("A block is in the box I guess!", "Running the intake backwards now and lifting");
                        telemetry.addData("Is pressure plate touched or color sensor?", drive.IntakeTouch1.isPressed() || drive.IntakeTouch2.isPressed()||drive.IntakeColor.alpha()>100);
                        telemetry.addData("Is the distance sensor close?", drive.Distance.getDistance(DistanceUnit.INCH) < distanceSensitivity);
//                        telemetry.addData("Was pressure plate touched?", hasABlock);
//                        telemetry.addData("Was the distance sensor close?", donewithIntake);
                        drive.Lift.setPower(LiftSpeed);
                        telemetry.update();
                    } else {
                        drive.Lift.setPower(0);
                    }

                    //When everything is done it dumps
                    if (!drive.isBusy()) {
                        drive.Indexer.setPosition(BoxDumpingPosition); //Dumps
                        drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                        drive.Intake.setPower(0);
                        AmountofBlocksGotten++;

                        drive.Lift.setDirection(DcMotorSimple.Direction.FORWARD);
                        drive.Lift.setPower(-LiftSpeed);
                        hasABlock = false;
                        donewithIntake = false;
                        //gets position
                        poseEstimate = drive.getPoseEstimate();
                        if(AmountofBlocksGotten==2) //just did first after scan:
                        {

                            //Goes further the second time we pick up a block
                            TrajectorySequence DumpLowerGoToWarehouse_2 = drive.trajectorySequenceBuilder(poseEstimate)
                                    .setReversed(false)
                                    .splineTo(new Vector2d(25,65),Math.toRadians(2)) //At Barrier, off by 1.5 in in y
                                    .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes+5,65.5),Math.toRadians(0))

                                    .setConstraints(drive.getVelocityConstraint(maxVel/Divider, Math.toRadians(maxAngularVel/Divider), 15.53),drive.getAccelerationConstraint(maxAccel))
                                    .splineToConstantHeading(new Vector2d(HowfarIntakeGoes+5,65.5),Math.toRadians(0))
//                                .splineTo(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))
                                    .build();
                            drive.followTrajectorySequenceAsync(DumpLowerGoToWarehouse_2);
                            currentState = State.DumpLowerGoToWarehouse;
                        }
                        else if (AmountofBlocksGotten==3) //If this was second after scan:
                        {
                            //Goes further the second time we pick up a block
                            TrajectorySequence DumpLowerGoToWarehouse_3 = drive.trajectorySequenceBuilder(poseEstimate)
                                    .setReversed(false)
                                    .splineTo(new Vector2d(25,65),Math.toRadians(2)) //At Barrier, off by 1.5 in in y
                                    .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes+10,65.5),Math.toRadians(0))

                                    .setConstraints(drive.getVelocityConstraint(maxVel/Divider, Math.toRadians(maxAngularVel/Divider), 15.53),drive.getAccelerationConstraint(maxAccel))
                                    .splineToConstantHeading(new Vector2d(HowfarIntakeGoes+10,65.5),Math.toRadians(0))
//                                .splineTo(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))
                                    .build();
                            drive.followTrajectorySequenceAsync(DumpLowerGoToWarehouse_3);
                            currentState = State.DumpLowerGoToWarehouse;
                        }
                        else if (AmountofBlocksGotten==4) //If this was third after scan:
                        {
                            //Goes further the second time we pick up a block
                            TrajectorySequence DumpLowerGoToWarehouse_4 = drive.trajectorySequenceBuilder(poseEstimate)
                                    .setReversed(false)
                                    .splineTo(new Vector2d(25,65),Math.toRadians(2)) //At Barrier, off by 1.5 in in y
                                    .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes+10,65.5),Math.toRadians(0))

                                    .setConstraints(drive.getVelocityConstraint(maxVel/Divider, Math.toRadians(maxAngularVel/Divider), 15.53),drive.getAccelerationConstraint(maxAccel))
                                    .splineToConstantHeading(new Vector2d(HowfarIntakeGoes+10,65.5),Math.toRadians(0))
//                                  .splineTo(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))
                                    .build();
                            drive.followTrajectorySequenceAsync(DumpLowerGoToWarehouse_4);
                            currentState = State.DumpLowerGoToWarehouse;
                        }
                        else { //parking:
                            Park = drive.trajectorySequenceBuilder(poseEstimate)
                                    .setReversed(false)
                                    .splineTo(new Vector2d(25,65),Math.toRadians(2)) //At Barrier, off by 1.5 in in y
                                    .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes+15,65.5),Math.toRadians(0))
                                    .build();
                            //Amount of blocks gotten = 1 follow normal trajectory
                            drive.followTrajectorySequenceAsync(DumpLowerGoToWarehouse);
                            currentState = State.Park;
                        }
                    }
                    break;
                case Park:
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