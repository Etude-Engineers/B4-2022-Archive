/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
modification,
 * are permitted (subject to the limitations in the disclaimer below)
provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be
used to endorse or
 * promote products derived from this software without specific prior
written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive;


//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.hardware.SensorColor;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.arcrobotics.ftclib.controller.PIDFController;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a
 single robot.
 */
public class HardwareTeleop
{
    public double doorUp = 0.75;
    public double doorDown = 0.5;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    /* Public OpMode members. */

    //Drive Motors:
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;


    //Scoring Freight:
    //Intake:
    public DcMotor Intake = null;
    public Servo IntakeDoor = null;

    //Box:
    public Servo Indexer = null;
    //Lift:
    public DcMotor Lift = null;

    public Servo CapstoneArm = null;
    public Servo CapstoneHand = null;

    //Engame:
    public DcMotor Spinner = null;
    public DcMotor TapeMotor = null;

    //Game Piece Servos:
//  public CRServo tapetheta = null;
//  public CRServo tapephi = null;
    public Servo tapetheta = null;
    public Servo tapephi = null;
    public Servo tapephi2 = null;

    //Sensors:
    //Lift:
    public TouchSensor Touchy = null;

    //Intake:
    public DistanceSensor Distance = null;
    public TouchSensor IntakeTouch1 = null;
    public TouchSensor IntakeTouch2 = null;

    //Auto:
    public ColorSensor Color = null;
    public ColorSensor Color2 = null;
    public ColorSensor IntakeColor = null;
    public RevBlinkinLedDriver blinkinLedDriver = null;


    /* local OpMode members. */


    /* Constructor */
    public HardwareTeleop(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap)
    {
        // Save reference to Hardware map
        this.hwMap = hwMap;
        // Define and Initialize Motors

        //Drive Motors:
        FL  = hwMap.get(DcMotor.class, "FL");
        BL = hwMap.get(DcMotor.class, "BL");
        FR  = hwMap.get(DcMotor.class, "FR");
        BR = hwMap.get(DcMotor.class, "BR");

        CapstoneArm = hwMap.get(Servo.class,"CapstoneArm");
        CapstoneHand = hwMap.get(Servo.class,"CapstoneHand");

        //Scoring Freight:
        //Intake:
        Intake  = hwMap.get(DcMotor.class, "Intake");
        IntakeDoor = hwMap.get(Servo.class, "IntakeDoor");

        //Box:
        Indexer = hwMap.get(Servo.class,"Indexer");

        Lift = hwMap.get(DcMotor.class,"Lift");


        //Endgame:
        Spinner = hwMap.get(DcMotor.class,"Spinner");


        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);

        //Sensors:
        Touchy = hwMap.get(TouchSensor.class, "Touchy");
        IntakeTouch1 = hwMap.get(TouchSensor.class, "IntakeTouch1");
        IntakeTouch2 = hwMap.get(TouchSensor.class, "IntakeTouch2");
        Color = hwMap.get(ColorSensor.class,"Color");
        Color2 = hwMap.get(ColorSensor.class,"Color2");
        IntakeColor = hwMap.get(ColorSensor.class, "IntakeColor");
        Distance = hwMap.get(DistanceSensor.class,"Distance");

        //Tape Servos:
//      tapetheta = hwMap.get(CRServo.class,"tapetheta");
//      tapephi = hwMap.get(CRServo.class,"tapephi");
        tapetheta = hwMap.get(Servo.class,"tapetheta");
        tapephi = hwMap.get(Servo.class,"tapephi");
        tapephi2 = hwMap.get(Servo.class,"tapephi2");
        TapeMotor = hwMap.get(DcMotor.class,"TapeMotor");

        //Sensors:

//
        //sensortouch = hwMap.get(TouchSensor.class,"sensor_touch");
//        Vsensor = hwMap.voltageSensor.iterator().next();
//        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");
//        blinkinLedDriver2 = hwMap.get(RevBlinkinLedDriver.class, "blinkin2");


        //Initialize Robot:

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Lift.setDirection(DcMotor.Direction.FORWARD);
        Spinner.setDirection(DcMotor.Direction.FORWARD);
//        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Tape Servos:
//        tapetheta.setPower(0);
//        tapephi.setPower(0);
        tapetheta.setPosition(0.5);
        tapephi.setPosition(0.5);
        tapephi2.setPosition(0.5);

        TapeMotor.setPower(0);

        // Set all motors to zero power
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        Intake.setPower(0);
        Indexer.setPosition(0.7); //Middle Position
//        Lift.setPower(0);
        Spinner.setPower(0);
        IntakeDoor.setPosition(doorUp);//Up

        CapstoneArm.setPosition(0.12);//up
        CapstoneHand.setPosition(0.6);//closed
    }
}
