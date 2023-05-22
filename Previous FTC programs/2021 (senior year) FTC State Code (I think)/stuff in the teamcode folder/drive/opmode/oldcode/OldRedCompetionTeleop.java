// /Copyright (c) 2017 FIRST. All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without modification,
//  * are permitted (subject to the limitations in the disclaimer below) provided that
//  * the following conditions are met:
//  *
//  * Redistributions of source code must retain the above copyright notice, this list
//  * of conditions and the following disclaimer.
//  *
//  * Redistributions in binary form must reproduce the above copyright notice, this
//  * list of conditions and the following disclaimer in the documentation and/or
//  * other materials provided with the distribution.
//  *
//  * Neither the name of FIRST nor the names of its contributors may be used to endorse or
//  * promote products derived from this software without specific prior written permission.
//  *
//  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
//  * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
//  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  */

package org.firstinspires.ftc.teamcode.drive.opmode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.HardwareTeleop;
import org.firstinspires.ftc.teamcode.drive.Mecanum;


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

@TeleOp(name="OldRedCompetionTeleop", group="Pushbot")
@Disabled
public class OldRedCompetionTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTeleop robot = new HardwareTeleop();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    boolean GIVETHISVARIABLEABETTERNAMEBECAUSETHISNAMEISABADNAMEFORAVARIABLETOHAVE = false;

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
    //RevBlinkinLedDriver.BlinkinPattern pattern;
    //RevBlinkinLedDriver.BlinkinPattern pattern2;

    public DistanceSensor Distance = null;
    public ColorSensor Color = null;
    public TouchSensor Touchy = null;
    double liftspeed = 0.8;

    double KAddGainPhi = 0.024;
    double KAddGainTheta = 0.004;
    double TapeSpeed = 1;

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

        boolean endgame = false;




        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        /*telemetry.addData("Horizontal Lift Position",  "Position = %3d", robot.HorizontalLift.getCurrentPosition());
        telemetry.update();*/

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad2.x) {
                endgame=true;
            }
            else if(gamepad2.y)
            {
                endgame=false;
            }

            if(endgame) //Engame Mode:
            {
                if (gamepad2.right_bumper)
                {
                    KAddGainTheta = .003;
                    KAddGainPhi = .001;
                    TapeSpeed = 0.33;
//
//                    robot.tapetheta.setPosition(robot.tapetheta.getPosition()+KAddGainPhi*gamepad2.left_stick_x);
//                    robot.tapephi.setPosition(robot.tapephi.getPosition()-KAddGainTheta*gamepad2.right_stick_y);
//                    robot.tapephi2.setPosition(robot.tapephi2.getPosition()+KAddGainTheta*gamepad2.right_stick_y);
////
//                    if(gamepad2.left_trigger>0)
//                    {
//                        robot.TapeMotor.setPower(gamepad2.left_trigger*TapeSpeed);
//                    }
//                    else
//                    {
//                        robot.TapeMotor.setPower(-gamepad2.right_trigger*TapeSpeed);
//
//                    }
                }
                else
                // if (!gamepad2.left_bumper&&gamepad2.right_bumper)
                {
                    KAddGainPhi = 0.024;
                    KAddGainTheta = 0.004;
                    TapeSpeed = 1;
                }
                //Tape Servos:
//                robot.tapetheta.setPower(gamepad2.left_stick_y);
//                robot.tapephi.setPower(gamepad2.right_stick_y);
                robot.tapetheta.setPosition(robot.tapetheta.getPosition()+KAddGainPhi*gamepad2.left_stick_x);
                robot.tapephi.setPosition(robot.tapephi.getPosition()-KAddGainTheta*gamepad2.right_stick_y);
                robot.tapephi2.setPosition(robot.tapephi2.getPosition()+KAddGainTheta*gamepad2.right_stick_y);

                if(gamepad2.left_trigger>0)
                {
                    robot.TapeMotor.setPower(gamepad2.left_trigger*TapeSpeed);
                }
                else
                {
                    robot.TapeMotor.setPower(-gamepad2.right_trigger*TapeSpeed);

                }
                telemetry.addData("Endgame!", "SPIN THOSE DUCKS");
                telemetry.addData("Up and Down Position!", robot.tapetheta.getPosition());
                telemetry.addData("Side to Side original Position!", robot.tapephi.getPosition());
                telemetry.addData("Side to Side new Position!", robot.tapephi2.getPosition());

                telemetry.update();
            }
            //final SensorColor sensorColor = new SensorColor((ColorSensor) hardwareMap);

            if (gamepad2.dpad_left&&!endgame) {
                //starting position
                robot.Indexer.setPosition(0.55);
            }
            else if(gamepad1.dpad_left&&!endgame)
            {
                robot.Indexer.setPosition(0.55);//shouldnt be needed anyways
            }
            else if (gamepad2.dpad_up&&!endgame) {
                //position while on lift
                robot.Indexer.setPosition(0.7);
            }
            else if (gamepad1.dpad_up&&!endgame) {
                //position while on lift
                robot.Indexer.setPosition(0.7);//shouldnt be needed anyways
            }
            else if (gamepad1.left_bumper&&!endgame) {
                robot.Indexer.setPosition(BoxDumpingPosition);
                sleep(600);
                robot.Indexer.setPosition(BoxLiftingPosition);
                LiftDown();
            }
            else if(gamepad2.dpad_down&&!endgame) {
                robot.Indexer.setPosition(BoxDumpingPosition);
                sleep(600);
                robot.Indexer.setPosition(BoxLiftingPosition);
                LiftDown();
            }
            else if(gamepad2.dpad_right&&!endgame)
            {
                robot.Indexer.setPosition(BoxDumpingPosition);
                sleep(600);
                robot.Indexer.setPosition(BoxLiftingPosition);
            }
                    else if(gamepad1.right_bumper&&!endgame)
                {
                    robot.Indexer.setPosition(BoxDumpingPosition);
                    sleep(600);
                    robot.Indexer.setPosition(BoxLiftingPosition);
                    }

            //Automated Control of Indexer:
            else if (gamepad2.left_stick_y != 0&&!endgame) {
                robot.Indexer.setPosition(BoxLiftingPosition);
                }
            else if (gamepad2.right_stick_y != 0&&!endgame&&robot.Touchy.isPressed()) {
                robot.Indexer.setPosition(BoxStartingPosition);
                robot.Intake.setPower(1);
            }
            //Lift base Controls: On Controller (could be used)
            else if (gamepad2.right_trigger > 0&&!endgame) {
                robot.Intake.setPower(-gamepad2.right_trigger);
            }
            else{
                robot.Intake.setPower(0);
            }

//
//            //Automated Controls of the lift to the right heights:
//            if (gamepad2.left_bumper&&!endgame) {
//                telemetry.addData("Starting:", "Lift 3 levels");
//                telemetry.update();
//                Lift3Levels(); //Eventually switch this to left bumper and make 1 level right bumper???
//            } else if (gamepad2.right_bumper&&!endgame) {
//                telemetry.addData("Starting:", "Lift 1 level");
//                telemetry.update();
//                LiftToFirst();
//            }
//            //Automated Control of the Box and Lift back down
//            else if (gamepad1.left_bumper) {
//                telemetry.addData("Starting:", "Lift Down");
//                telemetry.update();
//                //DumpAndLower3();
//            }
//            else if (gamepad1.right_bumper) {
//                telemetry.addData("Starting:", "Lift Down");
//                telemetry.update();
//                //DumpAndLower1();//           }

                robot.Lift.setPower(-gamepad2.left_stick_y * liftspeed);


            //robot.Spinner.setPower(-gamepad2.left_trigger * 1); //Driver completely controlling Spinner and speed probably pretty useful at least rn
            //Going to need a button at some point though if we make it's method
            if (gamepad2.y) {
                robot.Spinner.setPower(-0.5);
            }
            else if(gamepad2.x)
            {
                robot.Spinner.setPower(0.5);
            }
            else {
                robot.Spinner.setPower(-gamepad1.right_trigger*.5); //Driver completely controlling Spinner and speed probably pretty useful at least rn
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
        robot.FL.setPower(wheels.frontLeft * DrivePower);
        robot.FR.setPower(wheels.frontRight * DrivePower);
        robot.BL.setPower(wheels.backLeft * DrivePower);
        robot.BR.setPower(wheels.backRight * DrivePower);
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
//            if(gamepad2.right_bumper)
//            {
//                stopped=true;
//            }
        robot.Indexer.setPosition(BoxLiftingPosition);
        robot.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double liftTargetPos = 560;
        robot.Lift.setPower(.6);
        while (robot.Lift.getCurrentPosition() < liftTargetPos && !stopped) {
            telemetry.addData("Height:", robot.Lift.getCurrentPosition());
            telemetry.update();
            setDriveForMecanum(Mecanum.joystickToMotion(
                    gamepad1.left_stick_x, -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x, -gamepad1.right_stick_y));
            //wait for it to finish
//                if(gamepad2.right_bumper)
//                {
//                    stopped=true;
//                    break;
//                }
        }
        robot.Lift.setPower(0);
        robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Ready to Dump", "Ready to Dump");
        telemetry.update();

    }

    private void Lift3Levels() {
        boolean stopped = false;
        telemetry.addData("Starting:", "Lift to 3rd Level");
        telemetry.update();
//            if(gamepad2.left_bumper)
//            {
//                stopped=true;
//                break;
//            }
        robot.Indexer.setPosition(BoxLiftingPosition);
        robot.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Lift.setPower(.6);
        robot.Lift.setTargetPosition(1680);
        robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.Lift.isBusy() && !stopped) {
            telemetry.addData("Height:", robot.Lift.getCurrentPosition());
            telemetry.update();
            setDriveForMecanum(Mecanum.joystickToMotion(
                    gamepad1.left_stick_x, -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x, -gamepad1.right_stick_y));
            //wait for it to finish
//                if(gamepad2.left_bumper)
//                {
//                    stopped=true;
//                    break;
//                }
        }
        robot.Lift.setPower(0);
        robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Ready to Dump", "Ready to Dump");
        telemetry.update();

    }

//    private void DumpAndLower1() {
//        boolean stopped = false;
//            telemetry.addData("Starting:", "Dump & Lower");
//            telemetry.update();
////            if (gamepad2.dpad_down) {
////                stopped = true;
////                break;
////            }
//            //position while dumped
//            robot.Indexer.setPosition(BoxDumpingPosition);
//            sleep(600);
//            robot.Indexer.setPosition(BoxLiftingPosition);
//            robot.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.Lift.setPower(.6);
//            robot.Lift.setTargetPosition(-400);
//            robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (robot.Lift.isBusy() && !stopped) {
//                telemetry.addData("Height:", robot.Lift.getCurrentPosition());
//                telemetry.update();
//                setDriveForMecanum(Mecanum.joystickToMotion(
//                        gamepad1.left_stick_x, -gamepad1.left_stick_y,
//                        -gamepad1.right_stick_x, -gamepad1.right_stick_y));
//                //wait for it to finish
////                if(gamepad2.dpad_down)
////                {
////                    stopped=true;
////                    break;
////                }
//            }
//            robot.Lift.setPower(0);
//            robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.Indexer.setPosition(BoxStartingPosition);
//            telemetry.addData("Ready to Reload", "");
//            telemetry.update();
//        }
//
//
//    private void DumpAndLower3() {
//        boolean stopped = false;
//            telemetry.addData("Starting:", "Dump & Lower");
//            telemetry.update();
////            if (gamepad2.dpad_down) {
////                stopped = true;
////                break;
////            }
//            //position while dumped
//            robot.Indexer.setPosition(BoxDumpingPosition);
//            sleep(600);
//            robot.Indexer.setPosition(BoxLiftingPosition);
//            robot.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.Lift.setPower(.6);
//            robot.Lift.setTargetPosition(-1200);
//            robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (robot.Lift.isBusy() && !stopped) {
//                telemetry.addData("Height:", robot.Lift.getCurrentPosition());
//                telemetry.update();
//                setDriveForMecanum(Mecanum.joystickToMotion(
//                        gamepad1.left_stick_x, -gamepad1.left_stick_y,
//                        -gamepad1.right_stick_x, -gamepad1.right_stick_y));
//                //wait for it to finish
////                if(gamepad2.dpad_down)
////                {
////                    stopped=true;
////                    break;
////                }
//            }
//            robot.Lift.setPower(0);
//            robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.Indexer.setPosition(BoxStartingPosition);
//            telemetry.addData("Ready to Reload", "");
//            telemetry.update();
//        }
    public void LiftDown(){
        while (!robot.Touchy.isPressed()) {
            robot.Lift.setPower(-liftspeed); //0.8
            setDriveForMecanum(Mecanum.joystickToMotion(
                    gamepad1.left_stick_x, -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x, -gamepad1.right_stick_y));
        }
    }
    }

