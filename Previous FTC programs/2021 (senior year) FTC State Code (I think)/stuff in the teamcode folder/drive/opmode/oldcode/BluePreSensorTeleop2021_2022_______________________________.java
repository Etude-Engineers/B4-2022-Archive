package org.firstinspires.ftc.teamcode.drive.opmode.oldcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


//import org.firstinspires.ftc.teamcode.control.Mecanum;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="BlueAutoTeleop1", group="Pushbot")
@Disabled
public class BluePreSensorTeleop2021_2022_______________________________ extends LinearOpMode {

    /* Declare OpMode members. */
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    private ElapsedTime runtime = new ElapsedTime();

    Pose2d currentPosition = new Pose2d();

    static final double COUNTS_PER_MOTOR_REV = 4;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 1.6875;     // For figuring circumference
    static final double HORZ_LIFT_MAX_INCHES = 12.375;
    final double BoxStartingPosition = 0.55;
    final double BoxLiftingPosition = 0.7;
    final double BoxDumpingPosition = 1;
    //static final double MAX_HORZ_LIFT_POSITION = (HORZ_LIFT_MAX_INCHES/(WHEEL_DIAMETER_INCHES*3.1415))*COUNTS_PER_MOTOR_REV;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    //static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder

    //boolean rampUp  = true;

    //static final double H_LIFT_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /(WHEEL_DIAMETER_INCHES * 3.1415);
    //double HORZ_POSITION;


    // A timer helps provide feedback while calibration is taking place
    // ElapsedTime timer = new ElapsedTime();
    // private ElapsedTime     gametime = new ElapsedTime();
    double DrivePower = 1.0;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    RevBlinkinLedDriver.BlinkinPattern pattern2;

    public DistanceSensor Distance = null;
    public ColorSensor Color = null;
    public TouchSensor Touchy = null;
    double liftspeed = 0.8;


    @Override
    public void runOpMode() {
        Color = hardwareMap.get(ColorSensor.class, "Color");
        Distance = hardwareMap.get(DistanceSensor.class, "Distance");
        Touchy = hardwareMap.get(TouchSensor.class, "Touchy");

        // boolean lastResetState = false;
        // boolean curResetState  = false;
        double Power = 1;

       /*double InitPositonC = 0;
       double FinalPositonFA = 0;
       double FinalPositonC = 0.35;
       double InitPositonFA = 0.45;*/
        //Boolean intakeOveride =false;






        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap);
        drive.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
       /*telemetry.addData("Horizontal Lift Position",  "Position = %3d", robot.HorizontalLift.getCurrentPosition());
       telemetry.update();*/

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //final SensorColor sensorColor = new SensorColor((ColorSensor) hardwareMap);

            if (gamepad2.dpad_left) {
                //starting position
                drive.Indexer.setPosition(0.55);
            }
            else if(gamepad1.dpad_left)
            {
                drive.Indexer.setPosition(0.55);//shouldnt be needed anyways
            }
            else if (gamepad2.dpad_up) {
                //position while on lift
                drive.Indexer.setPosition(0.7);
            }
            else if (gamepad1.dpad_up) {
                //position while on lift
                drive.Indexer.setPosition(0.7);//shouldnt be needed anyways
            }
            else if (gamepad1.left_bumper) {
                drive.Indexer.setPosition(BoxDumpingPosition);
                sleep(600);
                drive.Indexer.setPosition(BoxLiftingPosition);
                LiftDown();
            }
            else if(gamepad2.dpad_down) {
                drive.Indexer.setPosition(BoxDumpingPosition);
                sleep(600);
                drive.Indexer.setPosition(BoxLiftingPosition);
                LiftDown();
            }
            else if(gamepad2.dpad_right)
            {
                drive.Indexer.setPosition(BoxDumpingPosition);
                sleep(600);
                drive.Indexer.setPosition(BoxLiftingPosition);
            }
            else if(gamepad1.right_bumper)
            {
                drive.Indexer.setPosition(BoxDumpingPosition);
                sleep(600);
                drive.Indexer.setPosition(BoxLiftingPosition);
            }
            if(gamepad1.x){
                currentPosition = drive.getPoseEstimate();
                TrajectorySequence toWarehouseEntrance = drive.trajectorySequenceBuilder(currentPosition)
                        .splineToSplineHeading(new Pose2d(21.5, 64, Math.toRadians(0)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(42,64),Math.toRadians(0))
                        .build();
            }
            if(gamepad1.y){
                currentPosition = drive.getPoseEstimate();
            }

            //Automated Control of Indexer:
            else if (gamepad2.left_stick_y != 0) {
                drive.Indexer.setPosition(BoxLiftingPosition);
            }
            else if (gamepad2.right_stick_y != 0) {
                drive.Indexer.setPosition(BoxStartingPosition);
                drive.Intake.setPower(1);
            }
            //Lift base Controls: On Controller (could be used)
            else if (gamepad2.right_trigger > 0) {
                drive.Intake.setPower(-gamepad2.right_trigger);
            }
            else{
                drive.Intake.setPower(0);
            }


            //Automated Controls of the lift to the right heights:
            if (gamepad2.left_bumper) {
                telemetry.addData("Starting:", "Lift 3 levels");
                telemetry.update();
                Lift3Levels(); //Eventually switch this to left bumper and make 1 level right bumper???
            } else if (gamepad2.right_bumper) {
                telemetry.addData("Starting:", "Lift 1 level");
                telemetry.update();
                LiftToFirst();
            }

            else {
                drive.Lift.setPower(-gamepad2.left_stick_y * liftspeed);
            }

            //Going to need a button at some point though if we make it's method
            if (gamepad2.y) {
                drive.Spinner.setPower(-0.5);
            }
            else if(gamepad2.x)
            {
                drive.Spinner.setPower(0.5);
            }
            else {
                drive.Spinner.setPower(gamepad1.right_trigger*.5); //Driver completely controlling Spinner and speed probably pretty useful at least rn
                telemetry.addData("Speed:", -gamepad1.right_trigger*.5);
                telemetry.update();
            }


            //Driving
            setDriveForMecanum(Mecanum.joystickToMotion(
                    gamepad1.left_stick_x, -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x, -gamepad1.right_stick_y));
        }
    }

    private void setDriveForMecanum(Mecanum.Motion motion) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
        drive.leftFront.setPower(wheels.frontLeft * DrivePower);
        drive.rightFront.setPower(wheels.frontRight * DrivePower);
        drive.leftRear.setPower(wheels.backLeft * DrivePower);
        drive.rightRear.setPower(wheels.backRight * DrivePower);
        telemetry.addData("FL", wheels.frontLeft * DrivePower);
        telemetry.addData("FR", wheels.frontRight * DrivePower);
        telemetry.addData("BL",wheels.backLeft * DrivePower);
        telemetry.addData("BR", wheels.backRight * DrivePower);
        telemetry.update();
    }

    int CountsPerRotationLift = 1120;
    int CountsPerlevel = 560;

    private void LiftToFirst() {
        boolean stopped = false;
        telemetry.addData("Starting:", "Lift to First Level");
        telemetry.update();

        drive.Indexer.setPosition(BoxLiftingPosition);
        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double liftTargetPos = 560;
        drive.Lift.setPower(.6);
        while (drive.Lift.getCurrentPosition() < liftTargetPos && !stopped) {
            telemetry.addData("Height:", drive.Lift.getCurrentPosition());
            telemetry.update();
            setDriveForMecanum(Mecanum.joystickToMotion(
                    gamepad1.left_stick_x, -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x, -gamepad1.right_stick_y));

        }
        drive.Lift.setPower(0);
        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Ready to Dump", "Ready to Dump");
        telemetry.update();

    }

    private void Lift3Levels() {
        boolean stopped = false;
        telemetry.addData("Starting:", "Lift to 3rd Level");
        telemetry.update();

        drive.Indexer.setPosition(BoxLiftingPosition);
        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.Lift.setPower(.6);
        drive.Lift.setTargetPosition(1680);
        drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (drive.Lift.isBusy() && !stopped) {
            telemetry.addData("Height:", drive.Lift.getCurrentPosition());
            telemetry.update();
            setDriveForMecanum(Mecanum.joystickToMotion(
                    gamepad1.left_stick_x, -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x, -gamepad1.right_stick_y));

        }
        drive.Lift.setPower(0);
        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Ready to Dump", "Ready to Dump");
        telemetry.update();

    }


    public void LiftDown(){
        while (!drive.Touchy.isPressed()) {
            drive.Lift.setPower(-liftspeed); //0.8
            setDriveForMecanum(Mecanum.joystickToMotion(
                    gamepad1.left_stick_x, -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x, -gamepad1.right_stick_y));
        }
    }
}
