package org.firstinspires.ftc.teamcode.drive.opmode.roadrunnerteleops;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */

@TeleOp(group = "drive")
@Disabled
public class BlueRoadrunnerCappingTeleop extends LinearOpMode {

//    HardwareTeleop robot = new HardwareTeleop();   // Use a Pushbot's hardware
    final double BoxStartingPosition = 0.55;
    final double BoxLiftingPosition = 0.7;
    final double BoxDumpingPosition = 1;

//    public DistanceSensor Distance = null;
//    public ColorSensor Color = null;
//    public TouchSensor Touchy = null;
    double liftspeed = 1;
    public Pose2d currentPosition;
    public Pose2d startingPosition = new Pose2d(-30.3, -61.75, Math.toRadians(270)); //If we Spin First, off by 1.5in
    boolean endgame = false;
    double KAddGainPhi = 0.024;
    double KAddGainTheta = 0.004;
    double TapeSpeed = 1;
    double Multiplyer = 3;

    //Where the tape measurer is on our robot, when the heading of the robot is 0
//    double distanceOffsetTape = 3.35; Sqrt(11.5)
//    double AngleOffsetTape = Math.toRadians(310);
    double xoffsetTape = -2.2;
    double yoffsetTape = 2.6;
//    double xoffsetTape = 0;
//    double yoffsetTape = 0;

    double distanceOffsetTape = Math.sqrt(xoffsetTape*xoffsetTape + yoffsetTape*yoffsetTape);
    double AngleOffsetTape =  Math.atan2(yoffsetTape,xoffsetTape);
    double HeightTapePos = 0; //CURRENTLY UNUSED

    //double phiInitialAngle = 0; //When robot heading is 0 Position
    //double phiAngleRange = 0; //From it's initial value (in the center of it, to its largest and smallest possible angle)

//Very Important:
    double thetaInitialAngle = Math.toRadians(315);  //If just the difference between it and the robot heading its 270. Position 0.5,
    double thetaAngleRange = 45*3*Math.PI/2*72;//270 degrees     From it's initial value (in the center of it, to its largest and smallest possible angle)


    double phiPos = 0.5; //From it's initial value (in the center of it, to its largest and smallest possible angle)
    double thetaPos = 0.5;  //From it's initial value (in the center of it, to its largest and smallest possible angle)

    //The allince hub position and height:
    double xAllainceHubPos = -11.5;
    double yAllainceHubPos =  23.75;
    double HeightAllainceHub =  20.3;

    //Based on our robot's current position and the position of the hub, where we need to aim if our tape measurer was in the center of our robot:
    double xRelativeRobot = 0;
    double yRelativeRobot = 0;

    double AnglFromRobotToAllaincehub = 0;
    double RelativeheadingRobot = 0;

    //Based on our robot's current position and the position of the hub, and where the tape measurer is on our robot we need to aim:
    double xRelativeTape = 0;
    double yRelativeTape = 0;
    double RelativeheadingTape = 0;

    int target = 0;

    //Positions for every tape spot:
    //-3: (Closest)
    double xDuckneg3 = -44.5;
    double yDuckneg3 = 35.75;
    //-2:
    double xDuckneg2 = -36;
    double yDuckneg2 = 35.75;
    //-1:
    double xDuckneg1 = -27.75;
    double yDuckneg1 = 35.75;
    //1:
    double xDuck1 = 2.75;
    double yDuck1 = 35.75;
    //2:
    double xDuck2 = 11.25;
    double yDuck2 = 35.75;
    //3 (Furthest)
    double xDuck3 = 19.5;
    double yDuck3 = 35.75;

    private ElapsedTime DumpTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
//        Color = hardwareMap.get(ColorSensor.class, "Color");
//        Distance = hardwareMap.get(DistanceSensor.class, "Distance");
//        Touchy = hardwareMap.get(TouchSensor.class, "Touchy");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            drive.update();
            currentPosition = drive.getPoseEstimate();
            telemetry.addData("PosX:", drive.getPoseEstimate().getX());
            telemetry.addData("PosY:", drive.getPoseEstimate().getY());
            drive.DrivePower = 1/(1+gamepad1.right_trigger*Multiplyer); //lowest speed is 1/1+multiplyer, rn 1/4

            if (drive.Distance.getDistance(DistanceUnit.INCH) < 1.5) {
                drive.Indexer.setPosition(BoxLiftingPosition);
            }

            if (gamepad2.x) {
                endgame = true;
            } else if (gamepad2.y) {
                endgame = false;
            }

            if (endgame) //Engame Mode:
            {
                if (gamepad2.right_trigger > 0) //Resets to corner of barriers in warehouse
                {
                    telemetry.addData("Set Position:", "At Spinner");
                    startingPosition = new Pose2d(-61.75, 62.825,Math.toRadians(90)); //(heading is ~PI/2)
                    drive.setPoseEstimate(startingPosition);
                }

                //If this works use dpad left and right to toggle between the 3 capstone positions and the alliance hub position, and either keep right trigger to go to it, or automatically go to it.
                if(gamepad1.right_bumper) //Goes to position above hub
                {
                    //Using Roadrunner coordinates and the alliance hub position figure out the relative triangle
                    xRelativeRobot = xAllainceHubPos - currentPosition.getX();
                    yRelativeRobot = yAllainceHubPos - currentPosition.getY();
                    AnglFromRobotToAllaincehub = -(Math.atan2(xRelativeRobot,yRelativeRobot)-0.5*Math.PI);

//                  RelativeheadingRobot = clampAngle(Math.atan2(xRelativeRobot,yRelativeRobot) -  currentPosition.getHeading());
                    RelativeheadingRobot = AnglFromRobotToAllaincehub - currentPosition.getHeading();

                    //Relative to the tape measurer:
                    xRelativeTape = xRelativeRobot+distanceOffsetTape*Math.cos(currentPosition.getHeading()+AngleOffsetTape);
                    yRelativeTape = yRelativeRobot+distanceOffsetTape*Math.sin(currentPosition.getHeading()+AngleOffsetTape);
                    RelativeheadingTape = clampAngle(fixHeading(RelativeheadingRobot-(AngleOffsetTape-Math.PI)));

                    //Prints out everything to double check it:
                    telemetry.addData("xRelativeTape: ", xRelativeTape);
                    telemetry.addData("yRelativeTape: ", yRelativeTape);
                    telemetry.addData("AngleOffsetTape: ", AngleOffsetTape);
                    telemetry.addData("RelativeheadingRobot: ", RelativeheadingRobot);
                    telemetry.addData("RelativeheadingTape: ", RelativeheadingTape);
                    telemetry.addData("Angle from center of robot allaince hub is:", -(Math.atan2(xRelativeRobot,yRelativeRobot)-0.5*Math.PI));
                    telemetry.addData("Heading of the robot: ", currentPosition.getHeading());
//
                    telemetry.addData("Setting to Angle:", RelativeheadingTape);
                    thetaPos = angletoPosition(RelativeheadingTape);
                    drive.tapetheta.setPosition(thetaPos);
                    telemetry.update();

                }
                if(gamepad1.dpad_right)
                {
                    if(target == 1)
                    {
                        target = -1;
                    }
                    else if(target>-3)
                    {
                        target--;
                    }

                    switch(target)
                    {
                        case -3:
                            //Using Roadrunner coordinates and the alliance hub position figure out the relative triangle
                            xRelativeRobot = xDuckneg3 - currentPosition.getX();
                            yRelativeRobot = yDuckneg3 - currentPosition.getY();

                            AnglFromRobotToAllaincehub = -(Math.atan2(xRelativeRobot,yRelativeRobot)-0.5*Math.PI);

//                          RelativeheadingRobot = clampAngle(Math.atan2(xRelativeRobot,yRelativeRobot) -  currentPosition.getHeading());
                            RelativeheadingRobot = AnglFromRobotToAllaincehub - currentPosition.getHeading();

                            //Relative to the tape measurer:
                            xRelativeTape = xRelativeRobot+distanceOffsetTape*Math.cos(currentPosition.getHeading()+AngleOffsetTape);
                            yRelativeTape = yRelativeRobot+distanceOffsetTape*Math.sin(currentPosition.getHeading()+AngleOffsetTape);
                            RelativeheadingTape = clampAngle(fixHeading(RelativeheadingRobot-(AngleOffsetTape-Math.PI)));

                            //Prints out everything to double check it:
                            telemetry.addData("xRelativeTape: ", xRelativeTape);
                            telemetry.addData("yRelativeTape: ", yRelativeTape);
                            telemetry.addData("AngleOffsetTape: ", AngleOffsetTape);
                            telemetry.addData("RelativeheadingRobot: ", RelativeheadingRobot);
                            telemetry.addData("RelativeheadingTape: ", RelativeheadingTape);
                            telemetry.addData("Angle from center of robot allaince hub is:", -(Math.atan2(xRelativeRobot,yRelativeRobot)-0.5*Math.PI));
                            telemetry.addData("Heading of the robot: ", currentPosition.getHeading());
//
                            telemetry.addData("Setting to Angle:", RelativeheadingTape);
                            thetaPos = angletoPosition(RelativeheadingTape);
                            drive.tapetheta.setPosition(thetaPos);
                            telemetry.update();

                            break;
                        case -2:
                        //Using Roadrunner coordinates and the alliance hub position figure out the relative triangle
                            xRelativeRobot = xDuckneg2 - currentPosition.getX();
                            yRelativeRobot = yDuckneg2 - currentPosition.getY();

                            AnglFromRobotToAllaincehub = -(Math.atan2(xRelativeRobot,yRelativeRobot)-0.5*Math.PI);

//                          RelativeheadingRobot = clampAngle(Math.atan2(xRelativeRobot,yRelativeRobot) -  currentPosition.getHeading());
                            RelativeheadingRobot = AnglFromRobotToAllaincehub - currentPosition.getHeading();

                            //Relative to the tape measurer:
                            xRelativeTape = xRelativeRobot+distanceOffsetTape*Math.cos(currentPosition.getHeading()+AngleOffsetTape);
                            yRelativeTape = yRelativeRobot+distanceOffsetTape*Math.sin(currentPosition.getHeading()+AngleOffsetTape);
                            RelativeheadingTape = clampAngle(fixHeading(RelativeheadingRobot-(AngleOffsetTape-Math.PI)));

                            //Prints out everything to double check it:
                            telemetry.addData("xRelativeTape: ", xRelativeTape);
                            telemetry.addData("yRelativeTape: ", yRelativeTape);
                            telemetry.addData("AngleOffsetTape: ", AngleOffsetTape);
                            telemetry.addData("RelativeheadingRobot: ", RelativeheadingRobot);
                            telemetry.addData("RelativeheadingTape: ", RelativeheadingTape);
                            telemetry.addData("Angle from center of robot allaince hub is:", -(Math.atan2(xRelativeRobot,yRelativeRobot)-0.5*Math.PI));
                            telemetry.addData("Heading of the robot: ", currentPosition.getHeading());
//
                            telemetry.addData("Setting to Angle:", RelativeheadingTape);
                            thetaPos = angletoPosition(RelativeheadingTape);
                            drive.tapetheta.setPosition(thetaPos);
                            telemetry.update();
                            break;
                        case -1:
                        //Using Roadrunner coordinates and the alliance hub position figure out the relative triangle
                            xRelativeRobot = xDuckneg1 - currentPosition.getX();
                            yRelativeRobot = yDuckneg1 - currentPosition.getY();

                            AnglFromRobotToAllaincehub = -(Math.atan2(xRelativeRobot,yRelativeRobot)-0.5*Math.PI);

                            //RelativeheadingRobot = clampAngle(Math.atan2(xRelativeRobot,yRelativeRobot) -  currentPosition.getHeading());
                            RelativeheadingRobot = AnglFromRobotToAllaincehub - currentPosition.getHeading();

                            //Relative to the tape measurer:
                            xRelativeTape = xRelativeRobot+distanceOffsetTape*Math.cos(currentPosition.getHeading()+AngleOffsetTape);
                            yRelativeTape = yRelativeRobot+distanceOffsetTape*Math.sin(currentPosition.getHeading()+AngleOffsetTape);
                            RelativeheadingTape = clampAngle(fixHeading(RelativeheadingRobot-(AngleOffsetTape-Math.PI)));

                            //Prints out everything to double check it:
                            telemetry.addData("xRelativeTape: ", xRelativeTape);
                            telemetry.addData("yRelativeTape: ", yRelativeTape);
                            telemetry.addData("AngleOffsetTape: ", AngleOffsetTape);
                            telemetry.addData("RelativeheadingRobot: ", RelativeheadingRobot);
                            telemetry.addData("RelativeheadingTape: ", RelativeheadingTape);
                            telemetry.addData("Angle from center of robot allaince hub is:", -(Math.atan2(xRelativeRobot,yRelativeRobot)-0.5*Math.PI));
                            telemetry.addData("Heading of the robot: ", currentPosition.getHeading());
                            telemetry.addData("Setting to Angle:", RelativeheadingTape);

                            thetaPos = angletoPosition(RelativeheadingTape);
                            drive.tapetheta.setPosition(thetaPos);
                            telemetry.update();
                            break;
                        case 1:
                            //Using Roadrunner coordinates and the alliance hub position figure out the relative triangle
                            xRelativeRobot = xDuck1 - currentPosition.getX();
                            yRelativeRobot = yDuck1 - currentPosition.getY();

                            AnglFromRobotToAllaincehub = -(Math.atan2(xRelativeRobot,yRelativeRobot)-0.5*Math.PI);

//                          RelativeheadingRobot = clampAngle(Math.atan2(xRelativeRobot,yRelativeRobot) -  currentPosition.getHeading());
                            RelativeheadingRobot = AnglFromRobotToAllaincehub - currentPosition.getHeading();

                            //Relative to the tape measurer:
                            xRelativeTape = xRelativeRobot+distanceOffsetTape*Math.cos(currentPosition.getHeading()+AngleOffsetTape);
                            yRelativeTape = yRelativeRobot+distanceOffsetTape*Math.sin(currentPosition.getHeading()+AngleOffsetTape);
                            RelativeheadingTape = clampAngle(fixHeading(RelativeheadingRobot-(AngleOffsetTape-Math.PI)));

                            //Prints out everything to double check it:
                            telemetry.addData("xRelativeTape: ", xRelativeTape);
                            telemetry.addData("yRelativeTape: ", yRelativeTape);
                            telemetry.addData("AngleOffsetTape: ", AngleOffsetTape);
                            telemetry.addData("RelativeheadingRobot: ", RelativeheadingRobot);
                            telemetry.addData("RelativeheadingTape: ", RelativeheadingTape);
                            telemetry.addData("Angle from center of robot allaince hub is:", -(Math.atan2(xRelativeRobot,yRelativeRobot)-0.5*Math.PI));
                            telemetry.addData("Heading of the robot: ", currentPosition.getHeading());
//
                            telemetry.addData("Setting to Angle:", RelativeheadingTape);
                            thetaPos = angletoPosition(RelativeheadingTape);
                            drive.tapetheta.setPosition(thetaPos);
                            telemetry.update();

                            break;
                        case 2:
                            //Using Roadrunner coordinates and the alliance hub position figure out the relative triangle
                            xRelativeRobot = xDuck2 - currentPosition.getX();
                            yRelativeRobot = yDuck2 - currentPosition.getY();

                            AnglFromRobotToAllaincehub = -(Math.atan2(xRelativeRobot,yRelativeRobot)-0.5*Math.PI);

//                          RelativeheadingRobot = clampAngle(Math.atan2(xRelativeRobot,yRelativeRobot) -  currentPosition.getHeading());
                            RelativeheadingRobot = AnglFromRobotToAllaincehub - currentPosition.getHeading();

                            //Relative to the tape measurer:
                            xRelativeTape = xRelativeRobot+distanceOffsetTape*Math.cos(currentPosition.getHeading()+AngleOffsetTape);
                            yRelativeTape = yRelativeRobot+distanceOffsetTape*Math.sin(currentPosition.getHeading()+AngleOffsetTape);
                            RelativeheadingTape = clampAngle(fixHeading(RelativeheadingRobot-(AngleOffsetTape-Math.PI)));

                            //Prints out everything to double check it:
                            telemetry.addData("xRelativeTape: ", xRelativeTape);
                            telemetry.addData("yRelativeTape: ", yRelativeTape);
                            telemetry.addData("AngleOffsetTape: ", AngleOffsetTape);
                            telemetry.addData("RelativeheadingRobot: ", RelativeheadingRobot);
                            telemetry.addData("RelativeheadingTape: ", RelativeheadingTape);
                            telemetry.addData("Angle from center of robot allaince hub is:", -(Math.atan2(xRelativeRobot,yRelativeRobot)-0.5*Math.PI));
                            telemetry.addData("Heading of the robot: ", currentPosition.getHeading());
//
                            telemetry.addData("Setting to Angle:", RelativeheadingTape);
                            thetaPos = angletoPosition(RelativeheadingTape);
                            drive.tapetheta.setPosition(thetaPos);
                            telemetry.update();
                            break;
                        case 3:
                            //Using Roadrunner coordinates and the alliance hub position figure out the relative triangle
                            xRelativeRobot = xDuck3 - currentPosition.getX();
                            yRelativeRobot = yDuck3 - currentPosition.getY();

                            AnglFromRobotToAllaincehub = -(Math.atan2(xRelativeRobot,yRelativeRobot)-0.5*Math.PI);

                            //RelativeheadingRobot = clampAngle(Math.atan2(xRelativeRobot,yRelativeRobot) -  currentPosition.getHeading());
                            RelativeheadingRobot = AnglFromRobotToAllaincehub - currentPosition.getHeading();

                            //Relative to the tape measurer:
                            xRelativeTape = xRelativeRobot+distanceOffsetTape*Math.cos(currentPosition.getHeading()+AngleOffsetTape);
                            yRelativeTape = yRelativeRobot+distanceOffsetTape*Math.sin(currentPosition.getHeading()+AngleOffsetTape);
                            RelativeheadingTape = clampAngle(fixHeading(RelativeheadingRobot-(AngleOffsetTape-Math.PI)));

                            //Prints out everything to double check it:
                            telemetry.addData("xRelativeTape: ", xRelativeTape);
                            telemetry.addData("yRelativeTape: ", yRelativeTape);
                            telemetry.addData("AngleOffsetTape: ", AngleOffsetTape);
                            telemetry.addData("RelativeheadingRobot: ", RelativeheadingRobot);
                            telemetry.addData("RelativeheadingTape: ", RelativeheadingTape);
                            telemetry.addData("Angle from center of robot allaince hub is:", -(Math.atan2(xRelativeRobot,yRelativeRobot)-0.5*Math.PI));
                            telemetry.addData("Heading of the robot: ", currentPosition.getHeading());
                            telemetry.addData("Setting to Angle:", RelativeheadingTape);

                            thetaPos = angletoPosition(RelativeheadingTape);
                            drive.tapetheta.setPosition(thetaPos);
                            telemetry.update();
                            break;
                    }
                    }

                else if(gamepad1.dpad_left)
                {
                    if(target == -1)
                    {
                        target = 1;
                    }
                    else if(target<3)
                    {
                        target++;
                    }
                    switch(target)
                    {
                        case 1:

                            break;
                        case 2:

                            break;
                        case 3:

                            break;
                    }
                }


                drive.setDriveForMecanum(Mecanum.joystickToMotion(
                        gamepad2.left_stick_x, -gamepad2.left_stick_y,
                        -gamepad2.right_stick_x, -gamepad2.right_stick_y));

                if (gamepad2.x) {
                    endgame = true;
                } else if (gamepad2.y) {
                    endgame = false;
                }

                drive.Spinner.setPower(gamepad2.right_trigger * .5);

                if (gamepad1.left_bumper) {
                    KAddGainTheta = .003;
                    KAddGainPhi = .001;
                    TapeSpeed = 0.33;


                    drive.tapetheta.setPosition(drive.tapetheta.getPosition() + KAddGainPhi * gamepad1.left_stick_x);
                    drive.tapephi.setPosition(drive.tapephi.getPosition() - KAddGainTheta * gamepad1.right_stick_y);
//                    drive.tapephi2.setPosition(drive.tapephi2.getPosition() + KAddGainTheta * gamepad1.right_stick_y);
                } else {
                    KAddGainPhi = 0.024;
                    KAddGainTheta = 0.004;
                    TapeSpeed = 1;
                }
                drive.tapetheta.setPosition(drive.tapetheta.getPosition() + KAddGainPhi * gamepad1.left_stick_x);
                drive.tapephi.setPosition(drive.tapephi.getPosition() - KAddGainTheta * gamepad1.right_stick_y);
//                drive.tapephi2.setPosition(drive.tapephi2.getPosition() + KAddGainTheta * gamepad1.right_stick_y);

                if (gamepad1.left_trigger > 0) {
                    drive.TapeMotor.setPower(gamepad1.left_trigger * TapeSpeed);
                } else {
                    drive.TapeMotor.setPower(-gamepad1.right_trigger * TapeSpeed);
                }
                telemetry.addData("Endgame!", "SPIN THOSE DUCKS");
                telemetry.addData("Up and Down Position!", drive.tapetheta.getPosition());
                telemetry.addData("Side to Side original Position!", drive.tapephi.getPosition());
//                telemetry.addData("Side to Side new Position!", drive.tapephi2.getPosition());

                telemetry.update();
            }
            else //Teleop:
            {
                if (gamepad1.start && gamepad1.left_bumper) //End of Warehouse Auto:
                {
                    telemetry.addData("Set Position:", "End of Warehouse Auto");
                    startingPosition = new Pose2d(44,65,Math.toRadians(0));
                    drive.setPoseEstimate(startingPosition);
                }
                else if (gamepad1.start && gamepad1.right_bumper) //End of Depot Auto:
                {
                    telemetry.addData("Set Position:", "End of Depot Auto");
                    startingPosition = new Pose2d(-60, 35.25, Math.toRadians(180));
                    drive.setPoseEstimate(startingPosition);
                }
                else if (gamepad1.start && gamepad1.left_trigger > 0) //Resets to corner of barriers in warehouse
                {
                    telemetry.addData("Set Position:", "Barrier's Corner");
                    startingPosition = new Pose2d(33.243,35.681,1.523); //(heading is ~PI/2)
                    drive.setPoseEstimate(startingPosition);
                }
                else if (gamepad1.start && gamepad1.right_trigger > 0) //Resets to near ducks
                {
                    telemetry.addData("Set Position:", "At Spinner");
                    startingPosition = new Pose2d(-61.75, -62.825,Math.toRadians(180)); //(heading is ~PI/2)
                    drive.setPoseEstimate(startingPosition);
                }
                telemetry.update();

//                if (gamepad1.right_bumper) {
//                    //Into Warehouse:
//                    TrajectorySequence IntoWarhouse = drive.trajectorySequenceBuilder(currentPosition)
//                            .splineToSplineHeading(new Pose2d(21.5, 64, Math.toRadians(0)), Math.toRadians(0))
//                            .splineToConstantHeading(new Vector2d(42, 64), Math.toRadians(0))
//                            .build();
//                    drive.followTrajectorySequenceAsync(IntoWarhouse);
//                } else if (gamepad1.left_bumper) {
//                    //Out of Warehouse:
//                    TrajectorySequence OutofWarehouse = drive.trajectorySequenceBuilder(currentPosition)
//                            .splineToSplineHeading(new Pose2d(21.5, 64, Math.toRadians(0)), Math.toRadians(0))
//                            .splineToConstantHeading(new Vector2d(42, 64), Math.toRadians(0))
//                            .build();
//                    drive.followTrajectorySequenceAsync(OutofWarehouse);
//                } else if (gamepad1.left_trigger>0.1) {
//                    //To Shared Hub Position
//                    TrajectorySequence ToSharedHub = drive.trajectorySequenceBuilder(currentPosition)
//                            .splineToSplineHeading(new Pose2d(21.5, 64, Math.toRadians(0)), Math.toRadians(0))
//                            .splineToConstantHeading(new Vector2d(42, 64), Math.toRadians(0))
//                            .build();
//                    drive.followTrajectorySequenceAsync(ToSharedHub);
//                } else if (gamepad1.right_trigger>0.1) {
//                    //Back to Warehouse from Shared Hub Position
//                    TrajectorySequence backfromSharedHub = drive.trajectorySequenceBuilder(currentPosition)
//                            .splineToSplineHeading(new Pose2d(21.5, 64, Math.toRadians(0)), Math.toRadians(0))
//                            .splineToConstantHeading(new Vector2d(42, 64), Math.toRadians(0))
//                            .build();
//                    drive.followTrajectorySequenceAsync(backfromSharedHub);
//                }


                if (gamepad2.dpad_left) {
                    //starting position
                    drive.Indexer.setPosition(0.55);
                }
                else if (gamepad2.dpad_up) {
                    //position while on lift
                    drive.Indexer.setPosition(0.7);
                }

                //Only need 1 of these 4, whichever is most comfortable:
                else if (gamepad2.dpad_down) {
                    drive.Indexer.setPosition(BoxDumpingPosition);
//                    sleep(500);
                    DumpTime.reset();
                    while (DumpTime.seconds()<0.5)
                    {
                        drive.setDriveForMecanum(Mecanum.joystickToMotion(
                                gamepad1.left_stick_x, -gamepad1.left_stick_y,
                                -gamepad1.right_stick_x, -gamepad1.right_stick_y));
                        drive.Lift.setPower(-gamepad2.left_stick_y * liftspeed); //Lifting
                    }
                    drive.Indexer.setPosition(BoxLiftingPosition);
                    //LiftDown();
                } else if (gamepad2.dpad_right) {
                    drive.Indexer.setPosition(BoxDumpingPosition);
//                    sleep(500);
                    DumpTime.reset();
                    while (DumpTime.seconds()<0.5)
                    {
                        drive.setDriveForMecanum(Mecanum.joystickToMotion(
                                gamepad1.left_stick_x, -gamepad1.left_stick_y,
                                -gamepad1.right_stick_x, -gamepad1.right_stick_y));
                        drive.Lift.setPower(-gamepad2.left_stick_y * liftspeed); //Lifting
                    }
                    drive.Indexer.setPosition(BoxLiftingPosition);
                } else if (gamepad2.right_bumper) {
                    drive.Indexer.setPosition(BoxDumpingPosition);
//                    sleep(500);
                    DumpTime.reset();
                    while (DumpTime.seconds()<0.5)
                    {
                        drive.setDriveForMecanum(Mecanum.joystickToMotion(
                                gamepad1.left_stick_x, -gamepad1.left_stick_y,
                                -gamepad1.right_stick_x, -gamepad1.right_stick_y));
                    }                    drive.Indexer.setPosition(BoxLiftingPosition);
                } else if (gamepad2.left_bumper) {
                    drive.Indexer.setPosition(BoxDumpingPosition);
//                    sleep(500);
                    DumpTime.reset();
                    while (DumpTime.seconds()<0.5)
                    {
                        drive.setDriveForMecanum(Mecanum.joystickToMotion(
                                gamepad1.left_stick_x, -gamepad1.left_stick_y,
                                -gamepad1.right_stick_x, -gamepad1.right_stick_y));
                    }                    drive.Indexer.setPosition(BoxLiftingPosition);
                }
                else if (gamepad1.dpad_down) {
                    drive.Indexer.setPosition(BoxDumpingPosition);
//                    sleep(500);
                    DumpTime.reset();
                    while (DumpTime.seconds()<0.5)
                    {
                        drive.setDriveForMecanum(Mecanum.joystickToMotion(
                                gamepad1.left_stick_x, -gamepad1.left_stick_y,
                                -gamepad1.right_stick_x, -gamepad1.right_stick_y));
                    }                    drive.Indexer.setPosition(BoxLiftingPosition);
                    //LiftDown();
                } else if (gamepad1.dpad_right) {
                    drive.Indexer.setPosition(BoxDumpingPosition);
//                    sleep(500);
                    DumpTime.reset();
                    while (DumpTime.seconds()<0.5)
                    {
                        drive.setDriveForMecanum(Mecanum.joystickToMotion(
                                gamepad1.left_stick_x, -gamepad1.left_stick_y,
                                -gamepad1.right_stick_x, -gamepad1.right_stick_y));
                    }                    drive.Indexer.setPosition(BoxLiftingPosition);
                } else if (gamepad1.right_bumper && !gamepad1.start) {
                    drive.Indexer.setPosition(BoxDumpingPosition);
//                    sleep(500);
                    DumpTime.reset();
                    while (DumpTime.seconds()<0.5)
                    {
                        drive.setDriveForMecanum(Mecanum.joystickToMotion(
                                gamepad1.left_stick_x, -gamepad1.left_stick_y,
                                -gamepad1.right_stick_x, -gamepad1.right_stick_y));
                        drive.Lift.setPower(-gamepad2.left_stick_y * liftspeed); //Lifting

                    }
                    drive.Indexer.setPosition(BoxLiftingPosition);
                } else if (gamepad1.left_bumper && !gamepad1.start) {
                    drive.Indexer.setPosition(BoxDumpingPosition);
//                    sleep(500);
                    DumpTime.reset();
                    while (DumpTime.seconds()<0.5)
                    {
                        drive.setDriveForMecanum(Mecanum.joystickToMotion(
                                gamepad1.left_stick_x, -gamepad1.left_stick_y,
                                -gamepad1.right_stick_x, -gamepad1.right_stick_y));
                        drive.Lift.setPower(-gamepad2.left_stick_y * liftspeed); //Lifting
                    }
                    drive.Indexer.setPosition(BoxLiftingPosition);
                }
                drive.Lift.setPower(-gamepad2.left_stick_y * liftspeed);


                //Automated Control of Indexer:
                if (gamepad2.left_stick_y != 0) {
                    drive.Indexer.setPosition(BoxLiftingPosition);
                } else if (gamepad2.right_stick_y != 0) {
                    drive.Indexer.setPosition(BoxStartingPosition);
                    drive.Intake.setPower(1);
                }
                //Lift base Controls: On Controller (could be used)
                else if (gamepad2.right_trigger > 0) {
                    drive.Intake.setPower(-gamepad2.right_trigger);
                } else {
                    drive.Intake.setPower(0);
                }

                //Driving
                drive.setDriveForMecanum(Mecanum.joystickToMotion(
                        gamepad1.left_stick_x, -gamepad1.left_stick_y,
                        -gamepad1.right_stick_x, -gamepad1.right_stick_y));

                telemetry.update();
            }
        }
    }

    private double clampAngle(double a)
    {
        return (a%2*Math.PI);
    }

    private double fixHeading(double a)
    {
        if(a<0) {
            return (a*-1)+Math.PI;
        }
        else
        {
            return (a);

        }
    }
    private double angletoPosition(double a)
    {
        double RelativeAngleFromInitial = (a-thetaInitialAngle);
        if((RelativeAngleFromInitial/2*thetaAngleRange +0.5)>=0 && (RelativeAngleFromInitial/2*thetaAngleRange +0.5)<=1) {
            telemetry.addData("Success! Setting angle to: ",(RelativeAngleFromInitial/2*thetaAngleRange +0.5));
//            telemetry.update();
            return (RelativeAngleFromInitial / 2 * thetaAngleRange + 0.5);
        }
        else if((RelativeAngleFromInitial/2*thetaAngleRange +0.5)<0)
        {
            telemetry.addData("Angle too small, trying to set servo to: ",(RelativeAngleFromInitial/2*thetaAngleRange +0.5));
//            telemetry.update();
            return 0;
        }
        else
        {
            telemetry.addData("Angle too big, trying to set servo to: ",(RelativeAngleFromInitial/2*thetaAngleRange +0.5));
//            telemetry.update();
            return 1;
        }
    }
}
