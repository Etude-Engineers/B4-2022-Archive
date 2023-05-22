package org.firstinspires.ftc.teamcode.drive.opmode.oldcode;

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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

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
@Disabled
public class RedWarehouseThriceAuto extends LinearOpMode {

    public RevBlinkinLedDriver blinkinLedDriver = null;
    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        ScanAndScore,   //First, go to Scan
        TRAJECTORY_1,   //First, go to Scan
        TRAJECTORY_2,   //Then, drop of block at Hub
        TRAJECTORY_3, //Then, go to warehouse and run the lift until we touch the touch sensor
        IntakeUntilTouchSensor, //Drive forward slowly and run the intake until we sense we have a block in our intake
        TRAJECTORY_4,   //Go back to hub, run the intake until the block is at the distance sensor, then lift up to the 3rd level.
        //and then run the lift until it's at the top
        IDLE            // Our bot will enter the IDLE state when done
    }
    final double BoxStartingPosition = 0.55;
    final double BoxLiftingPosition = 0.7;
    final double BoxDumpingPosition = 1;
    int level = 1;
    private ElapsedTime TotalTime = new ElapsedTime();
    private ElapsedTime FirstBlock = new ElapsedTime();
    double TookFirstLong = 15;
    private ElapsedTime SecondBlock = new ElapsedTime();
    double TookSecondLong = 15;
    private ElapsedTime ThirdBlock = new ElapsedTime();
    double TookThirdLong = 15;

    Boolean donewithIntake = false;
    Boolean hasABlock = false;
    Boolean hasDetected = false;

    double GoalBlocksToGet = 2;
    double WhenTooLittleTimeIsLeft = 26;

    private ElapsedTime detecttime = new ElapsedTime();
    private ElapsedTime lifttime = new ElapsedTime();
    private ElapsedTime Intaketime = new ElapsedTime();    // We define the current state we're on
    public double AmountofBlocksGotten = 0;

    double distanceSensitivity = 1.95;

    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startingPosition = new Pose2d(6.625, -58.75, Math.toRadians(270)); //If we Spin First, off by 1.5in

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our lift
        //Lift lift = new Lift(hardwareMap);

        // Initialize SampleMecanumDrive
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startingPosition);

        // Let's define our trajectories
        TrajectorySequence SpeedDetect = drive.trajectorySequenceBuilder(startingPosition) //Goes to Scan:
                .splineTo(new Vector2d(6.625, -45.675),Math.toRadians(270)) //y was like 4 inches too little, x was like an inch too little
                .splineTo(new Vector2d(-12.75, -39.25),Math.toRadians(270))
                .build();

        Trajectory trajectory1 = drive.trajectoryBuilder(startingPosition) //Goes to Scan:
                .lineToConstantHeading(new Vector2d(6.625, -44.675)) //y was like 4 inches too little, x was like an inch too little
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end()) //Goes to Hub:
                .strafeRight(12)
                .splineToConstantHeading(new Vector2d(-12.75, -39.25),Math.toRadians(270)) //seems like 1.5 inch off
                .build();

        // Third trajectory
        //Going to Warehouse
        // Ensure that we call trajectory2.end() as the start for this one
        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory2.end())
                .splineToLinearHeading(new Pose2d(8.5,-65, Math.toRadians(358)),Math.toRadians(358)) //At Barrier, off by 1.5 in in y
                .splineToConstantHeading(new Vector2d(44,-65),Math.toRadians(0))
                .build();
        // Fourth trajectory
        //Going back to hub
//        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
//                .lineToConstantHeading(new Vector2d(-15, 0))
//                .build();
        //To have the correct starting pos we have to make it below
        donewithIntake = false;
        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        TotalTime.reset();
        FirstBlock.reset();
        drive.Indexer.setPosition(BoxLiftingPosition);
        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.Lift.setDirection(DcMotor.Direction.FORWARD);
        detecttime.reset();

//        currentState = State.ScanAndScore; //Goes to blocks
//        drive.followTrajectorySequenceAsync(SpeedDetect);

        currentState = State.TRAJECTORY_1; //Goes to blocks
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
//                case ScanAndScore:
//                    if(!hasDetected) {
//                        if ((drive.Color.green() > 80)) //Needs to be at least >93
//                        {
//                            level = 3;
//                            telemetry.addData("Level:", level);
//                            telemetry.addData("Color:", drive.Color.green());
//                            telemetry.addData("Color2:", drive.Color2.green());
//                            telemetry.update();
//                            lifttime.reset();
//                            drive.Lift.setTargetPosition(1680);
//                            drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                            hasDetected = true;
//                        }
//                        else if ((drive.Color2.green() > 35)) //Needs to be at least >76
//                        {
//                            level = 2;
//                            telemetry.addData("Level:", level);
//                            telemetry.addData("Color:", drive.Color.green());
//                            telemetry.addData("Color2:", drive.Color2.green());
//                            telemetry.update();
//                            lifttime.reset();
//                            drive.Lift.setTargetPosition(1120);
//                            drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                            hasDetected = true;
//
//                        }
//                        else if (detecttime.seconds()>3){
//                            level = 1;
//                            telemetry.addData("I've decided it is level 1:", detecttime.seconds());
//                            telemetry.update();
//                            lifttime.reset();
//                            drive.Lift.setTargetPosition(560);
//                            drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                            hasDetected = false;
//                        }
//                        else {
//                            level = 1;
//                            telemetry.addData("So far level:", level);
//                            telemetry.addData("Color:", drive.Color.green());
//                            telemetry.addData("Color2:", drive.Color2.green());
//                            telemetry.update();
//                        }
//                    }
//                    else if(drive.isBusy())
//                        {
//                            drive.Indexer.setPosition(BoxDumpingPosition); //Dumps
//                            sleep(500);
//                            drive.Indexer.setPosition(BoxStartingPosition); //resets
//
//                            drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                            drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                            lifttime.reset();
//
//                            AmountofBlocksGotten = 1;
//                            TookFirstLong = FirstBlock.seconds();
//
//                            telemetry.addData("AutoTime:", TotalTime.seconds());
//                            telemetry.addData("First block took:", TookFirstLong);
//                            telemetry.update();
//                            SecondBlock.reset();
//                            telemetry.addData("Trajectory:", "3");
//
//                            drive.followTrajectorySequence(trajectory3); //Going back to warehouse
//                            currentState = State.TRAJECTORY_3;
//                        }
//                    break;
                case TRAJECTORY_1:
                    telemetry.addData("Trajectory", 1);
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) { //Once it gets to blocks detects the block and then starts the lift
//                        if (drive.Color.green() > 85) //Needs to be at least >93
//                        {
//                            level = 3;
//                            telemetry.addData("Level:", level);
//                            telemetry.addData("Color:", drive.Color.green());
//                            telemetry.addData("Color2:", drive.Color2.green());
//                            telemetry.update();
//                        } else if (drive.Color2.green() > 37.5) //Needs to be at least >76
//                        {
//                            level = 2;
//                            telemetry.addData("Level:", level);
//                            telemetry.addData("Color:", drive.Color.green());
//                            telemetry.addData("Color2:", drive.Color2.green());
//                            telemetry.update();
//                        } else {
//                            level = 1;
//                            telemetry.addData("Level:", level);
//                            telemetry.addData("Color:", drive.Color.green());
//                            telemetry.addData("Color2:", drive.Color2.green());
//                            telemetry.update();
//                        }
                        if(drive.Color.red()>60) //Needs to be at least >93
                        {
                            level = 2;
                            telemetry.addData("Level:", level);
                            telemetry.addData("Color:", drive.Color.green());
                            telemetry.addData("Color2:", drive.Color2.green());
                            telemetry.update();
                        }
                        else if (drive.Color2.red()>45) //Needs to be at least >76
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
                            telemetry.addData("Color:",drive.Color.green());
                            telemetry.addData("Color2:",drive.Color2.green());
                            telemetry.update();
                        }
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
                        lifttime.reset();
                        currentState = State.TRAJECTORY_2;
                        drive.followTrajectorySequenceAsync(trajectory2);
                    }
                    break;
                case TRAJECTORY_2:
                    telemetry.addData("Trajectory", 2);
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (drive.Lift.isBusy() && lifttime.seconds() < level + 0.5) { //Runs the lift to the right hight
                        telemetry.addData("Height:", drive.Lift.getCurrentPosition());
                        telemetry.update();
                        drive.Lift.setPower(1);
                    }

                    //Might want to make sure the Lift isnt busy here:
                    if (!drive.isBusy()) { //When finished (the lift should be done too) dump then go back to warehouse
                        drive.Indexer.setPosition(BoxDumpingPosition); //Dumps
                        sleep(500);
                        drive.Indexer.setPosition(BoxStartingPosition); //resets
                        drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

                        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lifttime.reset();

                        AmountofBlocksGotten = 1;
                        TookFirstLong = FirstBlock.seconds();

                        telemetry.addData("AutoTime:", TotalTime.seconds());
                        telemetry.addData("First block took:", TookFirstLong);
                        telemetry.update();
                        SecondBlock.reset();
                        telemetry.addData("Trajectory:", "3");

                        drive.followTrajectorySequence(trajectory3); //Going back to warehouse
                        currentState = State.TRAJECTORY_3;

                    }
                    break;
                case TRAJECTORY_3: //Goes to the warehouse, Lowers the lift
                    telemetry.addData("Trajectory", 3);
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished

                    if (!drive.Touchy.isPressed()) //If Lift isnt down, keeps doing down:
                    {
                        drive.Lift.setDirection(DcMotor.Direction.FORWARD);
                        telemetry.addData("Lift Going Down:", drive.Lift.getCurrentPosition());
                        telemetry.addData("Lift Speed:", -.6);
                        telemetry.addData("AutoTime:", TotalTime.seconds());
                        telemetry.addData("First block took:", TookFirstLong);
                        telemetry.update();
                        drive.Lift.setPower(-0.6);
                    } else //When down stops:
                    {
                        telemetry.addData("Lift Down:", drive.Lift.getCurrentPosition());
                        telemetry.addData("AutoTime:", TotalTime.seconds());
                        telemetry.addData("First block took:", TookFirstLong);
                        telemetry.update();
                        drive.Lift.setPower(0);
//                        drive.Intake.setPower(1);
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
                    if (!drive.isBusy() && AmountofBlocksGotten < GoalBlocksToGet) {
                        hasABlock = false;
                        donewithIntake = false;
                        currentState = State.IntakeUntilTouchSensor;
                    }
                    if (!drive.isBusy() && AmountofBlocksGotten > (GoalBlocksToGet - 1)) {
                        hasABlock = false;
                        donewithIntake = false;
                        currentState = State.IDLE;
                    }

                    break;
                case TRAJECTORY_4: //go back to hub, run the intake until the block is at the distance sensor, then runs the lift,
                    telemetry.addData("Trajectory", 4);
                    telemetry.addData("Is the distance sensor close?", drive.Distance.getDistance(DistanceUnit.INCH) < distanceSensitivity);
                    if (drive.Distance.getDistance(DistanceUnit.INCH) < distanceSensitivity) {
                        drive.Indexer.setPosition(BoxLiftingPosition);
                        drive.Intake.setPower(-1);
                        donewithIntake = true;
                        telemetry.addData("A block is in the box!", "Running the intake backwards now");
                        if (hasABlock) {
                            telemetry.addData("FYI:", "Pressure plate worked");
                        }
                        telemetry.update();
                    } else {
                        drive.Indexer.setPosition(BoxStartingPosition);
                        drive.Intake.setPower(1);
                    }
                    if (drive.Lift.getCurrentPosition() < (1680 + AmountofBlocksGotten * 50) && donewithIntake) {
                        telemetry.addData("A block is in the box I guess!", "Running the intake backwards now and lifting");
                        telemetry.addData("Is pressure plate touched or color sensor?", drive.IntakeTouch1.isPressed() || drive.IntakeTouch2.isPressed()||drive.IntakeColor.alpha()>100);
                        telemetry.addData("Is the distance sensor close?", drive.Distance.getDistance(DistanceUnit.INCH) < distanceSensitivity);
                        telemetry.addData("Was pressure plate touched?", hasABlock);
                        telemetry.addData("Was the distance sensor close?", donewithIntake);
                        drive.Lift.setPower(0.5);
                        telemetry.update();
                    } else {
                        drive.Lift.setPower(0);
                    }
                    if (!drive.isBusy()) {
                        drive.Indexer.setPosition(BoxDumpingPosition); //Dumps
                        sleep(500);
                        drive.Indexer.setPosition(BoxStartingPosition); //resets
                        drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                        drive.Intake.setPower(0);
                        AmountofBlocksGotten++;

                        drive.Lift.setDirection(DcMotorSimple.Direction.FORWARD);
                        drive.Lift.setPower(-0.6);

                        hasABlock = false;
                        donewithIntake = false;
                        if(AmountofBlocksGotten>1)
                        {
                            TrajectorySequence trajectory3_2 = drive.trajectorySequenceBuilder(trajectory2.end())
                                    .splineToLinearHeading(new Pose2d(8.5,-65.75, Math.toRadians(357)),Math.toRadians(357)) //At Barrier, off by 1.5 in in y
                                    .splineToConstantHeading(new Vector2d(46,-66),Math.toRadians(0))
                                    .build();
                            drive.followTrajectorySequenceAsync(trajectory3_2);
                            currentState = State.TRAJECTORY_3;
                        }
                        else {
                        drive.followTrajectorySequenceAsync(trajectory3);
                        currentState = State.TRAJECTORY_3;
                        }
                    }
                    break;
                case IntakeUntilTouchSensor:
                    telemetry.addData("Trajectory", "Intaking");
                    if (drive.Distance.getDistance(DistanceUnit.INCH) < distanceSensitivity) {
                        drive.Indexer.setPosition(BoxLiftingPosition);
                        drive.Intake.setPower(-1);

                        drive.setMotorPowers(0, 0, 0, 0);
                        donewithIntake = true;

                        telemetry.addData("A block is in the box!", "Running the intake backwards now");
                        telemetry.addData("Distance is:", drive.Distance.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    } else if (drive.IntakeTouch1.isPressed() || drive.IntakeTouch2.isPressed()||drive.IntakeColor.alpha()>100) {
                        drive.Indexer.setPosition(BoxStartingPosition);
                        drive.Intake.setPower(1);

                        drive.setMotorPowers(0, 0, 0, 0);
                        hasABlock = true;
                        drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);

                        telemetry.addData("A block touched my sensor!", "running the intake still");
                        telemetry.addData("Distance is:", drive.Distance.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    } else {
                        telemetry.addData("Distance is:", drive.Distance.getDistance(DistanceUnit.INCH));
                        telemetry.addData("Touched?", drive.IntakeTouch1.isPressed() || drive.IntakeTouch2.isPressed()||drive.IntakeColor.alpha()>100);
                        telemetry.update();

                        drive.Indexer.setPosition(BoxStartingPosition);
                        drive.Intake.setPower(1);
                        drive.setMotorPowers(.1, .1, .1, .1);
                    }
                    if ((donewithIntake || hasABlock) && AmountofBlocksGotten < 4 && TotalTime.seconds() < WhenTooLittleTimeIsLeft) { //If we still have time to keep going:
                        Pose2d poseEstimate = drive.getPoseEstimate();
                        TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(poseEstimate)
//                                .strafeLeft(2)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(5, -65, Math.toRadians(0)), Math.toRadians(180)) //Gets just outside of warehouse
//                                .splineTo(new Vector2d(-11.75, 44.25), Math.toRadians(-90))
                                .splineTo(new Vector2d(-11.75, -39.25), Math.toRadians(-270))
                                .build();

                        //telemetry.addData("Done, Touchy Says:",drive.Touchy.isPressed());
                        telemetry.addData("AutoTime:", TotalTime.seconds());
                        telemetry.addData("First block took:", TookFirstLong);

                        if (hasABlock) {
                            telemetry.addData("A block touched the touch sensor!", "Going to score ot now");
                            drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                        }

                        if (donewithIntake) {
                            telemetry.addData("A block is is in the box!", "running the intake backwards still");
                            drive.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        }
                        telemetry.addData("Distance is:", drive.Distance.getDistance(DistanceUnit.INCH));
                        telemetry.update();

                        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        drive.followTrajectorySequenceAsync(trajectory4);
                        currentState = State.TRAJECTORY_4;
                    }
                    if ((donewithIntake || hasABlock) && (AmountofBlocksGotten > 3 || TotalTime.seconds() >= WhenTooLittleTimeIsLeft)) {

                        telemetry.addData("Parking, got this many blocks:", AmountofBlocksGotten);
                        drive.setMotorPowers(0, 0, 0, 0);
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
                        else {
                            drive.Intake.setPower(0);
                            currentState = State.IDLE;
                        }
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