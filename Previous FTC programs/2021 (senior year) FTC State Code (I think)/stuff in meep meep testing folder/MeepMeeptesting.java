package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

import jdk.vm.ci.meta.Constant;


public class MeepMeeptesting {

    static double HowfarIntakeGoes = 55;
    static double HowfarIntoWarehouseRobotGoes = 47.5;
    public static double maxVel = 55;
    public static double maxAccel = 55;
    public static double Divider = 3;
    public static double BarrierEntrance = 20;
    //    public static double maxAngVel = 180;
    public static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(40, Math.toRadians(180),15.53);
    public static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(40);

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setDimensions(13.25,17.5)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(maxVel,  maxAccel, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(
//                                //Warehouse Start:
                                new Pose2d(12.5, 58.75, Math.toRadians(90)))
//                                //Duck Start:
//                               new Pose2d(-35.25, 58.75, Math.toRadians(90)))
//
//                                //Duck Auto:
//                                 .setReversed(true)
//                                .splineTo(new Vector2d(-58,30),Math.toRadians(270))
//                                .splineTo(new Vector2d(-28,25),Math.toRadians(0))
////                                .lineTo(new Vector2d(-11,41))
//                                .setReversed(false)
//                                .splineTo(new Vector2d(-28,25),Math.toRadians(180))
//                                .splineTo(new Vector2d(-61.75, 65.825),Math.toRadians(90)) //Goes like an inch less then it should, I think becuase it's so close
//
//
//
//
//                                //Warehouse Auto:
//                                .setReversed(true)
//                                .splineToLinearHeading(new Pose2d(-7, 41,Math.toRadians(75)),Math.toRadians(210)) //seems like 1.5 inch off
                                .lineTo(new Vector2d(-11,41))
                                .setReversed(false)
                                .splineTo(new Vector2d(20, 65), Math.toRadians(0)) //At Barrier, off by 1.5 in in y
//                                    .splineTo(new Vector2d(5,60),Math.toRadians(40))
//                                .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes,65),Math.toRadians(0))
                                .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes,65),Math.toRadians(0))
                                //angle can be not 0, but has to use the spline to constant heading then
//                                .setVelConstraint(getVelocityConstraint(maxVel/Divider, Math.toRadians(180), 15.53))
//                                .setReversed(false)
                                //If usees an angle other then 0 use this one instead:
                                .splineTo(new Vector2d(HowfarIntakeGoes,65),Math.toRadians(0)
                                        , getVelocityConstraint(maxVel/Divider, Math.toRadians(180/Divider), 15.53),getAccelerationConstraint(maxAccel)
                                )

                                //                                .splineTo(new Vector2d(0,35),Math.toRadians(230))
//                                .setReversed(false)
//                                .splineTo(new Vector2d(25,65),Math.toRadians(0)) //At Barrier, off by 1.5 in in y                                .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes,65.5),Math.toRadians(0))
//                                //angle can be not 0, but has to use the spline to constant heading then
//                                .setConstraints(getVelocityConstraint(maxVel/Divider, Math.toRadians(180/Divider), 15.53),getAccelerationConstraint(maxAccel))
//                                //If usees an angle other then 0 use this one instead:
//                                .splineToConstantHeading(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))
//                                .splineTo(new Vector2d(20, 65), Math.toRadians(0)) //At Barrier, off by 1.5 in in y
//                                .splineTo(new Vector2d(40,65),Math.toRadians(0)) //At Barrier, off by 1.5 in in y                                .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes,65.5),Math.toRadians(0))



                        //angle can be not 0, but has to use the spline to constant heading then
//                                .setConstraints(getVelocityConstraint(maxVel/Divider, Math.toRadians(180/Divider), 15.53),getAccelerationConstraint(maxAccel))
//                                //If usees an angle other then 0 use this one instead:
//                                .splineTo(new Vector2d(HowfarIntakeGoes,65),Math.toRadians(0))

//                                        .setConstraints(drive.getVelocityConstraint(maxVel / Divider, Math.toRadians(maxAngularVel / Divider), 15.53), drive.getAccelerationConstraint(maxAccel))
//                                .splineTo
//                                        (new Vector2d(HowfarIntakeGoes + 10, 65), Math.toRadians(0),
//                                                getVelocityConstraint(maxVel/Divider, Math.toRadians(180/Divider), 15.53),getAccelerationConstraint(maxAccel))

//                                .splineTo(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))


                                .setReversed(true)


                                .setVelConstraint(getVelocityConstraint(maxVel, Math.toRadians(180), 15.53))
                                .splineTo(new Vector2d(12,65),Math.toRadians(180)) //Gets just outside of warehouse
                                .splineTo(new Vector2d(-11, 41),Math.toRadians(-90))

                                //Cap Duck Auto:
//                                .setReversed(true)
                                //left
//                                .lineToConstantHeading(new Vector2d(-41.5, 45.675))
                                //middle
//                                .lineToConstantHeading(new Vector2d(-35.25, 45.675))
                                //almost works for right
//                                .splineTo(new Vector2d(-32.5, 45.175),Math.toRadians(315))


                                //Cap Warehouse Auto:
//                                .setReversed(true)
//                                .splineTo(new Vector2d(9, 45.175),Math.toRadians(315))
//                                .splineToLinearHeading(new Pose2d(-3, 40,Math.toRadians(60)),Math.toRadians(210)) //seems like 1.5 inch off
//
//                                //from here it loops: ammount of blocks gotten =1
//                                .setReversed(false)
//                                //16750's code:
//                                .splineTo(new Pose2d(BarrierEntrance, 66.5, Math.toRadians(0)).vec(),new Pose2d(BarrierEntrance, 66.5, Math.toRadians(0)).getHeading())
//                                .setVelConstraint((a, c, d, e)->20)
//                                .splineTo(new Pose2d(58, 67, Math.toRadians(0)).vec(),  new Pose2d(58, 67, Math.toRadians(0)).getHeading())
//
//                                //our code:
////                                .splineTo(new Vector2d(5,60),Math.toRadians(38))
////                                .splineTo(new Vector2d(15,65),Math.toRadians(0)) //At Barrier, off by 1.5 in in y
//
//                                //not used rn:
////                                .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes,65.5),Math.toRadians(0))
//                                //angle can be not 0, but has to use the spline to constant heading then
//
//                                //our code to get all the way into warehouse:
//                                .setConstraints(getVelocityConstraint(maxVel/Divider, Math.toRadians(180/Divider), 15.53),getAccelerationConstraint(maxAccel))
//                                //If usees an angle other then 0 use this one instead:
//                                .splineToConstantHeading(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))
////                                .splineTo(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))
//                                .setConstraints(getVelocityConstraint(maxVel, Math.toRadians(180), 15.53),getAccelerationConstraint(maxAccel))
//
//                                //our code back:
//                                .setReversed(true)
//                                .splineTo(new Vector2d(BarrierEntrance,65),Math.toRadians(180)) //Gets just outside of warehouse
//                                .splineTo(new Vector2d(-3, 40),Math.toRadians(240))
//                                //16750's code:
////                                .lineToLinearHeading(new Pose2d(BarrierEntrance, 66.5, Math.toRadians(0)))
////                                .splineTo(new Pose2d(-10, 42, Math.toRadians(65)).vec(), new Pose2d(-10, 42, Math.toRadians(65)).getHeading()+Math.PI)
//
//
//
//                                //start of 2nd loop ammount of blocks gotten =2
//                                .setReversed(false)
//                                .splineTo(new Vector2d(BarrierEntrance,65),Math.toRadians(0)) //At Barrier, off by 1.5 in in y
//                                .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes+5,65.5),Math.toRadians(0))
//
//                                .setConstraints(getVelocityConstraint(maxVel/Divider, Math.toRadians(180/Divider), 15.53),getAccelerationConstraint(maxAccel))
//                                .splineToConstantHeading(new Vector2d(HowfarIntakeGoes+5,65.5),Math.toRadians(0))
////                                .splineTo(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))
//
//                                .setConstraints(getVelocityConstraint(maxVel, Math.toRadians(180), 15.53),getAccelerationConstraint(maxAccel))
//                                .setReversed(true)
//                                .splineTo(new Vector2d(BarrierEntrance,65),Math.toRadians(180)) //Gets just outside of warehouse
//                                .splineTo(new Vector2d(-3, 40),Math.toRadians(240))
//
//
//
//                                //start of 3rd loop: ammount of blocks gotten =3
//                                .setReversed(false)
//                                .splineTo(new Vector2d(BarrierEntrance,65),Math.toRadians(2)) //At Barrier, off by 1.5 in in y
//                                .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes+10,65.5),Math.toRadians(0))
//
//                                .setConstraints(getVelocityConstraint(maxVel/Divider, Math.toRadians(180/Divider), 15.53),getAccelerationConstraint(maxAccel))
//                                .splineToConstantHeading(new Vector2d(HowfarIntakeGoes+10,65.5),Math.toRadians(0))
////                                .splineTo(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))
//
//                                .setConstraints(getVelocityConstraint(maxVel, Math.toRadians(180), 15.53),getAccelerationConstraint(maxAccel))
//                                .setReversed(true)
//                                .splineTo(new Vector2d(BarrierEntrance,65),Math.toRadians(180)) //Gets just outside of warehouse
//                                .splineTo(new Vector2d(-3, 40),Math.toRadians(240))
//
//
//
//                                //start of 4th loop: ammount of blocks gotten =4
//                                .setReversed(false)
//                                .splineTo(new Vector2d(BarrierEntrance,65),Math.toRadians(2)) //At Barrier, off by 1.5 in in y
//                                .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes+10,65.5),Math.toRadians(0))
//
//                                .setConstraints(getVelocityConstraint(maxVel/Divider, Math.toRadians(180/Divider), 15.53),getAccelerationConstraint(maxAccel))
//                                .splineToConstantHeading(new Vector2d(HowfarIntakeGoes+10,65.5),Math.toRadians(0))
////                                .splineTo(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))
//
//                                .setConstraints(getVelocityConstraint(maxVel, Math.toRadians(180), 15.53),getAccelerationConstraint(maxAccel))
//                                .setReversed(true)
//                                .splineTo(new Vector2d(BarrierEntrance,65),Math.toRadians(180)) //Gets just outside of warehouse
//                                .splineTo(new Vector2d(-3, 40),Math.toRadians(240))
//
//
//
//
//                                //parks ammount of blocks gotten =5
//                                .setReversed(false)
//                                .splineTo(new Vector2d(BarrierEntrance,65),Math.toRadians(2)) //At Barrier, off by 1.5 in in y
//                                .splineTo(new Vector2d(HowfarIntoWarehouseRobotGoes+15,65.5),Math.toRadians(0))
//
////                                .setConstraints(getVelocityConstraint(40/3, Math.toRadians(180/3), 15.53),getAccelerationConstraint(maxVel))
////                                .splineToConstantHeading(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))
//////                                .splineTo(new Vector2d(HowfarIntakeGoes,65.5),Math.toRadians(0))
////
////                                .setConstraints(getVelocityConstraint(40, Math.toRadians(180), 15.53),getAccelerationConstraint(40))
//                                //instead of doing another stops
////                                .setReversed(true)
////                                .splineTo(new Vector2d(12,65),Math.toRadians(180)) //Gets just outside of warehouse
////                                .splineTo(new Vector2d(-3, 40),Math.toRadians(240))
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}