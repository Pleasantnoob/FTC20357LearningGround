package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    // Match MainDrive / Auto2025 start poses
    static final double BLUE_START_X = -50.0, BLUE_START_Y = -50.0, BLUE_START_HEADING_DEG = -128.0;
    static final double RED_START_X = -50.0, RED_START_Y = 50.0, RED_START_HEADING_DEG = 128.0;
    static final double BLUE_FAR_START_X = 60.0, BLUE_FAR_START_Y = -10.0, BLUE_FAR_HEADING_DEG = 270.0;
    static final double RED_FAR_START_X = 60.0, RED_FAR_START_Y = 10.0, RED_FAR_HEADING_DEG = 90.0;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity blueNear = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.PI, Math.PI, 14.25)
                .build();
        RoadRunnerBotEntity blueFar = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.PI, Math.PI, 14.25)
                .build();
        RoadRunnerBotEntity redNear = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.PI, Math.PI, 14.25)
                .build();
        RoadRunnerBotEntity redFar = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.PI, Math.PI, 14.25)
                .build();

        blueNear.runAction(blueNear.getDrive().actionBuilder(new Pose2d(BLUE_START_X, BLUE_START_Y, Math.toRadians(BLUE_START_HEADING_DEG)))
                .strafeToLinearHeading(new Vector2d(-12,-12),Math.toRadians(270)) // Preload

                .strafeToLinearHeading(new Vector2d(-12,-28),Math.toRadians(270)) // first spike
                .strafeTo(new Vector2d(-12,-53)) // first spike intake
                .strafeToLinearHeading(new Vector2d(-12,-13), Math.toRadians(270)) //launch first spike
                        .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(12,-28,Math.toRadians(270)),Math.toRadians(270))// seconds spike
                //.strafeToLinearHeading(new Vector2d(-12,28, Math.toRadians(270))
                .strafeTo(new Vector2d(12,-53))//second spike intake
                        .setTangent(Math.toRadians(90))
                //.splineToLinearHeading(new Pose2d(-13,-12,Math.toRadians(270)),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-13,-12), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(12,-59),Math.toRadians(235))
                .strafeToLinearHeading(new Vector2d(-13,-12),Math.toRadians(250))
                .strafeToLinearHeading(new Vector2d(12,-59),Math.toRadians(235))
                .strafeToLinearHeading(new Vector2d(-35,-15),Math.toRadians(-128))
                


                .build());
        Pose2d blueFarStart = new Pose2d(BLUE_FAR_START_X, BLUE_FAR_START_Y, Math.toRadians(BLUE_FAR_HEADING_DEG));
        blueFar.runAction(blueFar.getDrive().actionBuilder(blueFarStart)
        //shoot
        //run intake
                .strafeToLinearHeading(new Vector2d(60, -62), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(50, -62), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(60, -50), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(60, -62), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(50, -62), Math.toRadians(270))
                //stop intake
                .strafeToLinearHeading(new Vector2d(60, -20), Math.toRadians(270), new TranslationalVelConstraint(100))
                //shoot
                .strafeTo(new Vector2d(60, -60))
                //run intake
                .strafeTo(new Vector2d(60, -20))
                //shoot
                .strafeTo(new Vector2d(60, -60))
                //run intake
                .strafeTo(new Vector2d(60, -20))
                //shoot
                //park
                .strafeTo(new Vector2d(50, -30))



                
                .build());
        // redNear = mirror of blueNear (Y flipped, 270→90, 315→45, 90→270, 235→125, 250→110, -128→128)
        redNear.runAction(redNear.getDrive().actionBuilder(new Pose2d(RED_START_X, RED_START_Y, Math.toRadians(RED_START_HEADING_DEG)))
                .strafeToLinearHeading(new Vector2d(-12, 12), Math.toRadians(90)) // Preload
                .strafeToLinearHeading(new Vector2d(-12, 28), Math.toRadians(90)) // first spike
                .strafeTo(new Vector2d(-12, 53)) // first spike intake
                .strafeToLinearHeading(new Vector2d(-12, 13), Math.toRadians(90)) // launch first spike
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(12, 28, Math.toRadians(90)), Math.toRadians(90)) // second spike
                .strafeTo(new Vector2d(12, 53)) // second spike intake
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-13, 12, Math.toRadians(90)), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(12, 59), Math.toRadians(125))
                .strafeToLinearHeading(new Vector2d(-13, 12), Math.toRadians(110))
                .strafeToLinearHeading(new Vector2d(12, 59), Math.toRadians(125))
                .strafeToLinearHeading(new Vector2d(-35, 15), Math.toRadians(128))
                .build());
        // redFar = mirror of blueFar (Y flipped)
        redFar.runAction(redFar.getDrive().actionBuilder(new Pose2d(RED_FAR_START_X, RED_FAR_START_Y, Math.toRadians(RED_FAR_HEADING_DEG)))
                .strafeToLinearHeading(new Vector2d(60, 62), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(50, 62), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(60, 50), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(60, 62), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(50, 62), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(60, 20), Math.toRadians(90), new TranslationalVelConstraint(100))
                .strafeTo(new Vector2d(60, 60))
                .strafeTo(new Vector2d(60, 20))
                .strafeTo(new Vector2d(60, 60))
                .strafeTo(new Vector2d(60, 20))
                .strafeTo(new Vector2d(50, 30))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueNear)
                .addEntity(blueFar)
                .addEntity(redNear)
                .addEntity(redFar)
                .start();
    }
}