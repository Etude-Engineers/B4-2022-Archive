/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode.teleop.TeleOp2023;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This OpMode scans a single servo back and forward until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a Robot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp

public class BLUETeleOp2023 extends LinearOpMode {
    //new
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.34;     // Minimum rotational position

    // Define class members
    Servo   servo1;
    Servo   servo2;
    Servo   servo3;
    TouchSensor touchSensor;
    double  position1;
    double  position2;
    double  position3 = 0.34;//Start at halfway position
    double Home = 0.34;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    // private DcMotor leftDrive = null;
    //  private DcMotor rightDrive = null;
    public DcMotor Lift;
    double Multi = -1;
    double Multi2 = 0.5;
    double Multi3 = 1;
    double position;

    boolean violet = false;



    // @Override
    public void runOpMode() {

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servo1 = hardwareMap.get(Servo.class, "Right");
        servo2 = hardwareMap.get(Servo.class, "Left");
        servo3 = hardwareMap.get(Servo.class, "Middle");
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        touchSensor = hardwareMap.get(TouchSensor.class,"TouchSensor");
        RevBlinkinLedDriver revBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class,"LED");
        revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the start button

        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive()){
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -(gamepad1.left_stick_y* Multi3),
//                            -(gamepad1.left_stick_x * Multi3),
//                            -(gamepad1.right_stick_x * Multi2)
//                    )
//            );
//            drive.update();
            double forward = -(gamepad1.left_stick_y* Multi3);
            double turn = -(gamepad1.right_stick_x * Multi2);
            double left = forward +turn;
            double right = forward-turn;
            double max = Math.max(Math.abs(left),Math.abs(right));
            if(max>1) {
                left/=max;
                right/=max;
            }
            drive.setMotorPowers(left,left,right,right);



            if (touchSensor.isPressed()){
                if(!violet){
                    violet = true;
                    revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                }
            }
            else {
                if(violet){
                    violet = false;
                    revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                }
            }

            position = (gamepad2.right_stick_y * Multi);

            Lift.setPower(position);

            position3 = java.lang.Math.abs(gamepad2.left_stick_x* Multi2);

            if ((position3 >= MIN_POS) && (position3 <= MAX_POS)) {

                servo3.setPosition(position3);
            }
            else {
                servo3.setPosition(Home);
            }


            if(gamepad2.right_bumper){
                Home = 0.34;
            }
            if(gamepad2.left_bumper){
                Home = 1;
            }

            if (Home == 1){

                position3 = -(java.lang.Math.abs(gamepad2.left_stick_x));
            }
            if (gamepad2.b) {
                servo1.setPosition(position1);
                servo2.setPosition(position2);

                position1 = 0.2;
                position2 = 0.4;
                servo1.setPosition(position1);
                servo2.setPosition(position2);
               /* sleep(2000);
                position1 = 0.5;
                position2 = 0.15;*/

                servo1.setPosition(position1);
                servo2.setPosition(position2);
                revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                //TODO: Fix Color:

            }
            if (gamepad2.a) {
//                servo1.setPosition(position1);
//                servo2.setPosition(position2);

                position1 = 0.58                                                                                                                                                                                                                            ;
//                position2 = 0.15;
                position2 = 0.1;
                servo1.setPosition(position1);
                servo2.setPosition(position2);
                /*sleep(2000);
                position2 = 0.5;
                position1 = 0.15;*/

//                servo1.setPosition(position1);
//                servo2.setPosition(position2);
                revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
                //TODO: Fix Color:
//                revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
            if(gamepad1.right_bumper){
                Multi3 = 0.5;
            }
            if(gamepad1.left_bumper){
                Multi3 = 1;
            }



            telemetry.addData("Lift",position);
            telemetry.addData("Middle", position3);
            telemetry.addData("Left Claw", position2);
            telemetry.addData("Right Claw", position1);

            telemetry.update();
        }
        // slew the servo, according to the rampUp (direction) variable.

        //  telemetry.addData("RS",gamepad2.right_stick_y);

        // Set the servo to the new position and pause;


    }


}


