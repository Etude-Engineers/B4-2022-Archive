package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.roadrunnerwebcam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.autonomous.webcam.FreightFrenzyDeterminationPipeline_Duck;
import org.firstinspires.ftc.teamcode.drive.opmode.teleop.LiftPID;
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
@Autonomous(group = "drive")
public class BlueDuckWebcamAutoNoCap extends LinearOpMode {

    //Webcam and detection initialization:
    OpenCvWebcam webcam;
    FreightFrenzyDeterminationPipeline_Duck pipeline;
    FreightFrenzyDeterminationPipeline_Duck.FreightPosition snapshotAnalysis = FreightFrenzyDeterminationPipeline_Duck.FreightPosition.LEFT; // default
    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
//    Trajectory DrivetoGamepeice;
//    TrajectorySequence DriveToHub;
//    TrajectorySequence DumpLowerGoToSpinDuck;
//    TrajectorySequence CollectDuck;
//    TrajectorySequence ScoreDuck;
//    TrajectorySequence Park;
    enum State {
        DriveToHub,   //Drives to the Hub and raises the lift to the right level and raises the capstone arm
        //Up to here has been tuned with meep meep
        DumpLowerGoToSpinDuck,   //Go to dumping position, start moving back to warehouse and then slightly after starting to move back
        //but the bucket back to lifting positon and lower the lift
//        IntakeUntilTouchSensor, //Drive forward slowly and run the intake until we sense we have a block in our intake
        SpinDuck,   //Go back to hub, run the intake until the block is at the distance sensor, then lift up to the 3rd level while intaking backwards.
//        CollectDuck,
//        ScoreDuck,
        Park,
        //After BacktoHub do DumpLowerGoToWarehouse again
        IDLE            // Our bot will enter the IDLE state when done
    }


    double LiftSpeed = 1;

    static double HowfarIntakeGoes = 55;
    static double HowfarIntoWarehouseRobotGoes = 45;

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
    //If this much time is left dont score anymore:
    double WhenTooLittleTimeIsLeft = 30;

    //Timers for the lift, detecting, Intake or Capstone if needed
    private ElapsedTime Detecttime = new ElapsedTime();
    private ElapsedTime Lifttime = new ElapsedTime();
    private ElapsedTime Intaketime = new ElapsedTime();
    private ElapsedTime Capstonetime = new ElapsedTime();
    private ElapsedTime Ducktime = new ElapsedTime();


    //Default to IDLE and resets variables
    State currentState = State.IDLE;
    int level = 3;
    int LiftPos = 1680;

    LiftPID CustomLiftPID = new  LiftPID(.001,0,0,0,0);


    //In the middle of the pieces
    Pose2d poseEstimate = new Pose2d(-35.25, 58.75, Math.toRadians(90));
    //TODO: Try inital pose
    Pose2d startingPosition = new Pose2d(-35.25, 58.75, Math.toRadians(90)); //If we Spin First, off by 1.5in

    @Override
    public void runOpMode() throws InterruptedException {

        //Detection Setup:
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new FreightFrenzyDeterminationPipeline_Duck();
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

        drive.setPoseEstimate(startingPosition);

        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Todo: Try Trajectory to hub
        TrajectorySequence DriveToHub = drive.trajectorySequenceBuilder(poseEstimate) //Goes to Hub:
                .setReversed(true)
                .splineTo(new Vector2d(-58,37.5),Math.toRadians(270))
                .splineTo(new Vector2d(-58,24),Math.toRadians(270))
                .splineTo(new Vector2d(-28.5,18),Math.toRadians(358))
                .build();


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
                    LiftPos = 480;
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

        //TODO: Set Lift levels:
        switch (snapshotAnalysis) {
            case LEFT: {
                level = 1;
                LiftPos = 480;
                break;
            }

            case CENTER: {
                level = 2;
                LiftPos = 1020;
                break;
            }
            case RIGHT: {
                level = 3;
                LiftPos = 1680;
                break;
            }
        }
            telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
            telemetry.addData("level:", level);
            telemetry.update();



        if (isStopRequested()) return;


        //Set the current state to DrivetoGamepeice, our first step, Then have it follow that trajectory
        //Because of async it lets the program continue
        Lifttime.reset();
        TotalTime.reset();
        FirstBlock.reset();
        drive.Indexer.setPosition(BoxLiftingPosition);
        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.Lift.setDirection(DcMotor.Direction.FORWARD);


        drive.followTrajectorySequenceAsync(DriveToHub);
        currentState = State.DriveToHub;
        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
                switch (currentState){

                case DriveToHub:
                    telemetry.addData("State: ", "DriveToHub");
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state once finished
//                    if (drive.Lift.getCurrentPosition()<LiftPos && Lifttime.seconds() < level + 0.5) { //Runs the lift to the right hight
//                        telemetry.addData("Going to:", LiftPos);
//                        telemetry.addData("Height:", drive.Lift.getCurrentPosition());
//                        telemetry.update();
//                        drive.Lift.setPower(1);
//                    }

                    if (level==1 && drive.Lift.getCurrentPosition()<480)
                    {
                        drive.Lift.setPower(1);
                    }
                    else if (level==1&&drive.Lift.getCurrentPosition()>=480)
                    {
                        drive.Lift.setPower(0);
                    }
                    if (level==2&& drive.Lift.getCurrentPosition()<1020)
                    {
                        drive.Lift.setPower(1);
                    }
                    else if (level==2&&drive.Lift.getCurrentPosition()>=1020)
                    {
                        drive.Lift.setPower(0);
                    }
                    if (level==3&& drive.Lift.getCurrentPosition()<1680)
                    {
                        drive.Lift.setPower(1);
                    }
                    else if (level==3&&drive.Lift.getCurrentPosition()>=1680)
                    {
                        drive.Lift.setPower(0);
                    }

                    //Might want to make sure the Lift isnt busy here:
                    if (!drive.isBusy()) { //When finished (the lift should be done too) dump then go back to warehouse

                        drive.Indexer.setPosition(BoxDumpingPosition); //Dumps
                        sleep(700);
                        drive.Indexer.setPosition(BoxStartingPosition); //Dumps
                        drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

                        //Prepares the lift for lowering:
//                        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Lifttime.reset();


                        //Sets the drive to go to the hub regardless where it is right now
                        poseEstimate = drive.getPoseEstimate();

                        //TODO: Tune Trajectory that goes to the Spinner
                        TrajectorySequence DumpLowerGoToSpinDuck = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(false)
                                .splineTo(new Vector2d(-50,20),Math.toRadians(180))
                                .splineTo(new Vector2d(-60, 54),Math.toRadians(90))
                                .build();
                            drive.followTrajectorySequenceAsync(DumpLowerGoToSpinDuck); //Going back to warehouse
                            Lifttime.reset();
                            currentState = State.DumpLowerGoToSpinDuck;
                    }
                    break;

                case DumpLowerGoToSpinDuck: //Goes to the Spin the Duck, Lowers the lift
                    telemetry.addData("State: ", "DumpLowerGoToSpinDuck");

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


                    if(!drive.isBusy()) {
                        Ducktime.reset();
                        currentState = State.SpinDuck;
                    }
                    break;

                case SpinDuck: //go back to hub, run the intake until the block is at the distance sensor, then runs the lift,
                    telemetry.addData("State: ", "SpinningDuck");
                    telemetry.addData("Is the distance sensor close?", drive.Distance.getDistance(DistanceUnit.INCH) < distanceSensitivity);
                        drive.Spinner.setPower(0.5);
//                        if(Ducktime.seconds()>1.25)
//                        {
//                            drive.Spinner.setPower(0.6);
//                        }
                        if(Ducktime.seconds()>4)
                        {
                            drive.Spinner.setPower(0);
//                            drive.Intake.setPower(1);

                            //TODO: Try Trajectory that collects the Duck
                            Pose2d DUCK_COLLECT1 = new Pose2d(-55, 59.5, Math.toRadians(90));
                            Pose2d DUCK_COLLECT2 = new Pose2d(-35, 59.5, Math.toRadians(90));
//                            TrajectorySequence CollectDuck = drive.trajectorySequenceBuilder(poseEstimate)
//
//                                    .lineToSplineHeading(new Pose2d(-55, 59.5, Math.toRadians(70)))
//                                    .lineTo(DUCK_COLLECT2.vec())
//                                    .turn(Math.toRadians(40))
//                                    .lineTo(DUCK_COLLECT1.vec())
//                                    .turn(Math.toRadians(-40))
//                                    .lineTo(DUCK_COLLECT2.vec())
////                                    .turn(Math.toRadians(40))
////                                    .lineTo(DUCK_COLLECT1.vec())
//                                    .build();
//                            drive.followTrajectorySequenceAsync(CollectDuck);
                            poseEstimate = drive.getPoseEstimate();
                            TrajectorySequence Park = drive.trajectorySequenceBuilder(poseEstimate)
//                                    .lineTo(new Vector2d(-50,25))
                                    .lineTo(new Vector2d(-64,32))
//                                .splineTo(new Vector2d(12,65),Math.toRadians(2)) //At Barrier, off by 1.5 in in y
                                    .build();
                            drive.followTrajectorySequenceAsync(Park);
                            currentState = State.Park;
                        }
                        break;
//                case CollectDuck: //go back to hub, run the intake until the block is at the distance sensor, then runs the lift,
//
//                    if(drive.IntakeColor.alpha()>100)
//                    {
//                        hasABlock = true;
//                    }
//                    if(drive.Distance.getDistance(DistanceUnit.INCH)>distanceSensitivity)
//                    {
//                        donewithIntake = true;
//                    }
//                    if(hasABlock||donewithIntake)
//                {
//                    poseEstimate = drive.getPoseEstimate();
//                    //ToDo: Try trajectory for Scoring the duck
//                    drive.Lift.setDirection(DcMotorSimple.Direction.FORWARD);
//                    TrajectorySequence ScoreDuck = drive.trajectorySequenceBuilder(poseEstimate)
//                            .splineTo(new Vector2d(-58,30),Math.toRadians(270))
//                            .splineTo(new Vector2d(-28,25),Math.toRadians(0))
//                            .build();
//                    drive.followTrajectorySequenceAsync(ScoreDuck);
//                    currentState = State.ScoreDuck;
//                }
//                    if(!drive.isBusy())
//                    {
//                        poseEstimate = drive.getPoseEstimate();
//                        //TODO: Try Trajectory that parks
//                        TrajectorySequence Park = drive.trajectorySequenceBuilder(poseEstimate)
//                                .lineTo(new Vector2d(-64,35))
//                                .build();
//                        drive.followTrajectorySequenceAsync(Park);
//                        currentState = State.Park;
//                    }
//                    break;
//                case ScoreDuck: //go back to hub, run the intake until the block is at the distance sensor, then runs the lift,
//                    telemetry.addData("State: ", "ScoreingDuck");
//
//                    if(drive.Lift.getCurrentPosition()<1680)
//                    {
//                        drive.Lift.setPower(LiftSpeed);
//                    }
//                    else
//                    {
//                        drive.Lift.setPower(0);
//                    }
//                    //Intakes until distance sensor:
//                     if (drive.Distance.getDistance(DistanceUnit.INCH) < distanceSensitivity) {
//                        drive.Indexer.setPosition(BoxLiftingPosition);
//                        donewithIntake = true;
//                         drive.Intake.setPower(0);
//                         drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//                    } else {
//                        drive.Indexer.setPosition(BoxStartingPosition);
//                        drive.Intake.setPower(1);
//                    }
//
//                     //When Intake is done runs the Lift to the top:
//                    if (drive.Lift.getCurrentPosition() < (1680) && donewithIntake) {
//                        drive.Lift.setPower(LiftSpeed);
//                    } else {
//                        drive.Lift.setPower(0);
//                    }
//
//                    //When everything is done it dumps
//                    if (!drive.isBusy()) {
//                        drive.Indexer.setPosition(BoxDumpingPosition); //Dumps
//                        sleep(600);
//                        drive.Indexer.setPosition(BoxLiftingPosition); //Dumps
//                        drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
//                        drive.Intake.setPower(0);
//                        drive.Lift.setDirection(DcMotorSimple.Direction.FORWARD);
//                        drive.Lift.setPower(-LiftSpeed);
//                        hasABlock = false;
//                        donewithIntake = false;
//
//                        //gets position
//                        poseEstimate = drive.getPoseEstimate();
//                        //TODO: Try Trajectory that parks
//                        TrajectorySequence Park = drive.trajectorySequenceBuilder(poseEstimate)
//                                .lineTo(new Vector2d(-64,35))
////                                .splineTo(new Vector2d(12,65),Math.toRadians(2)) //At Barrier, off by 1.5 in in y
//                                .build();
//
//                        drive.followTrajectorySequenceAsync(Park);
//                        currentState = State.Park;
//                    }
//                        break;
                        case Park:
                            if(!drive.isBusy())
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