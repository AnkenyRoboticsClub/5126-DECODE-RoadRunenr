package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-47, 47, Math.toRadians(130));
        Pose2d shootingRed = new Pose2d(-23, 23, Math.toRadians(130));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 9.24)
                .setDimensions(10.8, 18)
                .build();

        DriveShim drive = myBot.getDrive();

        Action lineUp = drive.actionBuilder(startPose)
                //.afterTime(0, launcher.setLauncherVelocity(1050))
                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(135))
                .build();

        Action BallGrab = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-15, 16), Math.toRadians(90))
                .build();


        Action fullAuto = new SequentialAction(
                //gate.setGatePosition(Gate.openPosition),
                lineUp,
                BallGrab,
                new SleepAction(2.0)
        );


        myBot.runAction(fullAuto);
        /*
        myBot.runAction(myBot.getDrive().actionBuilder(startPose)

                //.strafeTo(new Vector2d(-23, 23)) //Back to shoot
                lineUp
                //Get Ball 1 v
                .waitSeconds(1)
                .strafeTo(new Vector2d(-15, 16))//Got to get balls
                .turnTo(Math.toRadians(90))
                //shoot
                .strafeTo(new Vector2d(-11.8, 32))// Start of balls
                .strafeTo(new Vector2d(-11.8, 59))// Getting balls
                .strafeTo(new Vector2d(-11.8, 32))//comming back
                .splineTo(new Vector2d(-23, 23), Math.toRadians(310))// going to shoot
                //Get Ball 2 v
                .waitSeconds(1)
                .strafeTo(new Vector2d(12, 23))//2nd row
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(12, 59))// Getting balls
                .strafeTo(new Vector2d(12, 23))//comming back
                .splineTo(new Vector2d(-23, 23), Math.toRadians(310)) //Back to shoot
                        .strafeToConstantHeading(new Vector2d(12, 23))
                .build());
         */

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}