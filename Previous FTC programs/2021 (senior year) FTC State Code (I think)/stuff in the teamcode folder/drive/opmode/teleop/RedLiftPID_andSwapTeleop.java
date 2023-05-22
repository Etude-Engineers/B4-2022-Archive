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

package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.HardwareTeleop;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
//import com.arcrobotics.ftclib.controller

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

@TeleOp(name="RedLiftPID_andSwapTeleop", group="Pushbot")
//@Disabled
public class RedLiftPID_andSwapTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTeleop robot = new HardwareTeleop();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    public double doorUp = 0.75;
    public double doorDown = 0.5;
    double KAddGainPhi = 0.024;
    double KAddGainTheta = 0.004;
    double TapeSpeed = 1;

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
    private ElapsedTime DumpTime = new ElapsedTime();
    double DrivePower = 1.0;
    //RevBlinkinLedDriver.BlinkinPattern pattern;
    //RevBlinkinLedDriver.BlinkinPattern pattern2;

    double liftspeed = 1;
    boolean WeHaveAGamePiece = false;
    double distanceSensitivity = 1.95;
    double Multiplyer = 3;

    //What the lift should be at:
    double LiftTargetHieght = 0;
    ElapsedTime Lifttimer = new ElapsedTime();
    LiftPID CustomLiftPID = new  LiftPID(.0005,0,0,0,0);
//    PIDCoefficients LiftPID = new PIDCoefficients(1,0,0)
//    PIDFController  LiftPIDF = new PIDFController (LiftPID,0,,0,1);

    @Override
    public void runOpMode() {
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
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad2.left_bumper) //open
            {
                robot.CapstoneHand.setPosition(.375);//open
            }
            if(gamepad2.right_bumper) //closed
            {
                robot.CapstoneHand.setPosition(0.59);//closed
            }
            if(gamepad2.dpad_left) //initial up
            {
                robot.CapstoneArm.setPosition(0.2);//initial up
            }
            if(gamepad2.dpad_up) //storing peice
            {
                robot.CapstoneArm.setPosition(.3);//storing peice
            }
            if(gamepad2.dpad_right) //down
            {
                robot.CapstoneArm.setPosition(0.925);//down
            }
            double Gain = 0.025;
            if(gamepad2.left_trigger>0.05)
            {
                robot.CapstoneArm.setPosition(robot.CapstoneArm.getPosition()+gamepad2.right_stick_y*Gain);//down
            }

            telemetry.addData("Intake Color Sees Block?:",robot.IntakeColor.alpha()>100);
            //Always running:
            DrivePower = 1/(1+gamepad1.left_trigger*Multiplyer); //lowest speed is 1/1+multiplyer, rn 1/4
            if(robot.IntakeTouch1.isPressed()||robot.IntakeTouch2.isPressed()||robot.IntakeColor.alpha()>100)
            {
                robot.IntakeDoor.setPosition(doorDown);
                WeHaveAGamePiece = true;
            }
            if(robot.Distance.getDistance(DistanceUnit.INCH)<distanceSensitivity)
            {
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
            else if(WeHaveAGamePiece)
            {
                robot.Indexer.setPosition(BoxLiftingPosition);
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                telemetry.addData("we have a game piece", "");
                telemetry.update();
            }
            else
            {
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
            }

            if (gamepad2.x) {
                endgame=true;
            }
            else if(gamepad2.y)
            {
                endgame=false;
            }

            if(endgame) //Engame Mode:
            {
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
                setDriveForMecanum(Mecanum.joystickToMotion(
                        gamepad2.left_stick_x, -gamepad2.left_stick_y,
                        -gamepad2.right_stick_x, -gamepad2.right_stick_y));

                if (gamepad2.x) {
                    endgame=true;
                }
                else if(gamepad2.y)
                {
                    endgame=false;
                }

                robot.Spinner.setPower(-gamepad2.right_trigger*.5);

                if (gamepad1.left_bumper)
                {
                    KAddGainTheta = .003;
                    KAddGainPhi = .001;
                    TapeSpeed = 0.33;
                    robot.tapetheta.setPosition(robot.tapetheta.getPosition()+KAddGainPhi*gamepad1.left_stick_x);
                    robot.tapephi.setPosition(robot.tapephi.getPosition()-KAddGainTheta*gamepad1.right_stick_y);
                    robot.tapephi2.setPosition(robot.tapephi2.getPosition()+KAddGainTheta*gamepad1.right_stick_y);
                }
                else
                {
                    KAddGainPhi = 0.024;
                    KAddGainTheta = 0.004;
                    TapeSpeed = 1;
                }
                robot.tapetheta.setPosition(robot.tapetheta.getPosition()+KAddGainPhi*gamepad1.left_stick_x);
                robot.tapephi.setPosition(robot.tapephi.getPosition()-KAddGainTheta*gamepad1.right_stick_y);
                robot.tapephi2.setPosition(robot.tapephi2.getPosition()+KAddGainTheta*gamepad1.right_stick_y);

                    if(gamepad1.left_trigger>0)
                {
                    robot.TapeMotor.setPower(gamepad1.left_trigger*TapeSpeed);
                }
                else
                {
                    robot.TapeMotor.setPower(-gamepad1.right_trigger*TapeSpeed);

                }
                telemetry.addData("Endgame!", "SPIN THOSE DUCKS");
                telemetry.addData("Up and Down Position!", robot.tapetheta.getPosition());
                telemetry.addData("Side to Side original Position!", robot.tapephi.getPosition());
                telemetry.addData("Side to Side new Position!", robot.tapephi2.getPosition());

                telemetry.update();
            }

                else //Teleop:
            {
                robot.Spinner.setPower(-gamepad1.right_trigger*0.5);

//                if (gamepad2.dpad_left&&!endgame) {
//                    //starting position
//                    robot.Indexer.setPosition(0.55);
//                    robot.IntakeDoor.setPosition(doorUp);
//                    WeHaveAGamePiece = false;
//                }
                if(gamepad1.dpad_left&&!endgame)
                {
                    robot.Indexer.setPosition(0.55);//shouldnt be needed anyways
                }
//                else if (gamepad2.dpad_up&&!endgame) {
//                    //position while on lift
//                    robot.Indexer.setPosition(0.7);
//                    robot.IntakeDoor.setPosition(doorDown);
//                    WeHaveAGamePiece = true;
//                }
                else if (gamepad1.dpad_up&&!endgame) {
                    //position while on lift
                    robot.Indexer.setPosition(0.7);//shouldnt be needed anyways
                    robot.IntakeDoor.setPosition(doorDown);
                    WeHaveAGamePiece = true;
                }
//                else if(gamepad2.dpad_down&&!endgame) {
//                    robot.Indexer.setPosition(BoxDumpingPosition);
////                    sleep(500);
//                    DumpTime.reset();
//                    while (DumpTime.seconds()<0.5)
//                    {
//                        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
//                        setDriveForMecanum(Mecanum.joystickToMotion(
//                                gamepad1.left_stick_x, -gamepad1.left_stick_y,
//                                -gamepad1.right_stick_x, -gamepad1.right_stick_y));
//                        robot.Lift.setPower(-gamepad2.left_stick_y * liftspeed); //Lifting
//                        LiftTargetHieght = robot.Lift.getCurrentPosition();
//                    }
//                    robot.IntakeDoor.setPosition(doorUp);
//                    WeHaveAGamePiece = false;
//                    robot.Indexer.setPosition(BoxLiftingPosition);
//                  //  LiftDown();
//                }
//                else if(gamepad2.dpad_right&&!endgame)
//                {
//                    robot.Indexer.setPosition(BoxDumpingPosition);
////                    sleep(500);
//                    DumpTime.reset();
//                    while (DumpTime.seconds()<0.5)
//                    {
//                        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
//                        setDriveForMecanum(Mecanum.joystickToMotion(
//                                gamepad1.left_stick_x, -gamepad1.left_stick_y,
//                                -gamepad1.right_stick_x, -gamepad1.right_stick_y));
//                        robot.Lift.setPower(-gamepad2.left_stick_y * liftspeed); //Lifting
//                        LiftTargetHieght = robot.Lift.getCurrentPosition();
//                    }
//                    robot.IntakeDoor.setPosition(doorUp);
//                    WeHaveAGamePiece = false;
//                    robot.Indexer.setPosition(BoxLiftingPosition);
//                }
                else if(gamepad1.right_bumper&&!endgame)
                {
                    robot.Indexer.setPosition(BoxDumpingPosition);
//                    sleep(500);
                    DumpTime.reset();
                    while (DumpTime.seconds()<0.5)
                    {
                        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
                        setDriveForMecanum(Mecanum.joystickToMotion(
                                gamepad1.left_stick_x, -gamepad1.left_stick_y,
                                -gamepad1.right_stick_x, -gamepad1.right_stick_y));
                        robot.Lift.setPower(-gamepad2.left_stick_y * liftspeed); //Lifting
                        LiftTargetHieght = robot.Lift.getCurrentPosition();
                    }
                    robot.IntakeDoor.setPosition(doorUp);
                    WeHaveAGamePiece = false;
                    robot.Indexer.setPosition(BoxLiftingPosition);
                }
                else if (gamepad1.left_bumper&&!endgame) {
                    robot.Indexer.setPosition(BoxDumpingPosition);
//                    sleep(500);
                    DumpTime.reset();
                    while (DumpTime.seconds()<0.5)
                    {
                        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
                        setDriveForMecanum(Mecanum.joystickToMotion(
                                gamepad1.left_stick_x, -gamepad1.left_stick_y,
                                -gamepad1.right_stick_x, -gamepad1.right_stick_y));
                        robot.Lift.setPower(-gamepad2.left_stick_y * liftspeed); //Lifting
                        LiftTargetHieght = robot.Lift.getCurrentPosition();
                    }
                    robot.IntakeDoor.setPosition(doorUp);
                    WeHaveAGamePiece = false;
                    robot.Indexer.setPosition(BoxLiftingPosition);
                   // LiftDown();
                }

                //Automated Control of Indexer:
                else if (gamepad2.left_stick_y != 0&&!endgame) {
                    robot.Indexer.setPosition(BoxLiftingPosition);
                    robot.IntakeDoor.setPosition(doorDown);
                }
                //Intake Controls:
                else if (gamepad2.right_stick_y != 0&&!endgame&&(robot.Touchy.isPressed()||gamepad2.dpad_down)) {
                    if(robot.Distance.getDistance(DistanceUnit.INCH)<distanceSensitivity)
                    {
                        robot.Indexer.setPosition(BoxLiftingPosition); //Normal Intaking
                    }
                    else {
                        robot.Indexer.setPosition(BoxStartingPosition); //Normal Intaking
                    }
                    robot.Intake.setPower(1);
                }
                else if (gamepad2.right_trigger > 0&&!endgame) {
                    robot.Intake.setPower(-gamepad2.right_trigger); //Backwards
                }
                else{
                    robot.Intake.setPower(0);
                }



                if(gamepad2.left_stick_y>0&&robot.Touchy.isPressed())
                {
                    robot.Lift.setPower(0); //When we are all the way down we the intake automatically
                    LiftTargetHieght = robot.Lift.getCurrentPosition();
                }
                else if (-gamepad2.left_stick_y > 0.025 || -gamepad2.left_stick_y < -0.025) {
                    robot.Lift.setPower(-gamepad2.left_stick_y * liftspeed); //Lifting
                    LiftTargetHieght = robot.Lift.getCurrentPosition();
                }

                else
                {
                    robot.Lift.setPower(0); //Stopped Lift
                    if(CustomLiftPID.Calculate(robot.Lift.getCurrentPosition(),LiftTargetHieght,0.02)>0) {
                        robot.Lift.setPower(CustomLiftPID.Calculate(robot.Lift.getCurrentPosition(), LiftTargetHieght, 0.02));
                    }
//                    Lifttimer.reset();
                    telemetry.addData("Lift target Height:", LiftTargetHieght);
                    telemetry.addData("Lift Current Height:", robot.Lift.getCurrentPosition());
                    telemetry.addData("Lift PID value:", CustomLiftPID.Calculate(robot.Lift.getCurrentPosition(), LiftTargetHieght, 0.02));
                }


                telemetry.addData("Teleop!", "Get that freight");
                telemetry.addData("Distance", robot.Distance.getDistance(DistanceUnit.INCH));    //
                telemetry.update();

                setDriveForMecanum(Mecanum.joystickToMotion(
                        gamepad1.left_stick_x, -gamepad1.left_stick_y,
                        -gamepad1.right_stick_x, -gamepad1.right_stick_y));
            }


        }
    }

    private void setDriveForMecanum(Mecanum.Motion motion) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
        robot.FL.setPower(wheels.frontLeft * DrivePower);
        robot.FR.setPower(wheels.frontRight * DrivePower);
        robot.BL.setPower(wheels.backLeft * DrivePower);
        robot.BR.setPower(wheels.backRight * DrivePower);
    }

    int CountsPerRotationLift = 1120;
    int CountsPerlevel = 560;

    }

