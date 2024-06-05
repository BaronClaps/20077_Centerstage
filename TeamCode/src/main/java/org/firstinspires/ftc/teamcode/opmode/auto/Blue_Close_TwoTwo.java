package org.firstinspires.ftc.teamcode.opmode.auto;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.config.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.config.subsystem.*;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Blue Close 2+2", group = "Blue")
public class Blue_Close_TwoTwo extends LinearOpMode {

    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        ClawSubsystem claw = new ClawSubsystem(hardwareMap);
        LiftSubsystem lift = new LiftSubsystem(hardwareMap);
        GearRotationSubsystem gear = new GearRotationSubsystem(hardwareMap);
        Pose2d beginPose = new Pose2d(-62, 12, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        PresetSubsystem presets = new PresetSubsystem(claw, lift, gear);
        CameraSubsystem camera = new CameraSubsystem(hardwareMap);
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        camera.SPEED_GAIN = -0.02;   // Drive = Error * Gain
        camera.STRAFE_GAIN = 0.01;
        camera.TURN_GAIN = 0;

        // Actions that run when Init
        claw.closeClaws();
        gear.wheelServo_Deactivated();
        camera.initAprilTag();
        camera.setManualExposure(2, 50);

        //---------------------------------------- Blue Close 2+2 ----------------------------------------------\\
        Pose2d startPose = new Pose2d(-62, 12, 0);
        Pose2d yellowScoringPose1 = new Pose2d(-36, 30, Math.toRadians(270));
        Pose2d yellowScoringPose2 = new Pose2d(-30, 22, Math.toRadians(270));
        Pose2d yellowScoringPose3 = new Pose2d(-36, 8, Math.toRadians(270));
        Pose2d yellowScoringOverridePose1 = new Pose2d(-42, 45, Math.toRadians(270));
        Pose2d yellowScoringOverridePose2 = new Pose2d(-36, 45, Math.toRadians(270));
        Pose2d yellowScoringOverridePose3 = new Pose2d(-27, 45, Math.toRadians(270));
        Pose2d driveToWhitePose1 = new Pose2d(-42, 51.5, Math.toRadians(270));//-38
        Pose2d driveToWhitePose2 = new Pose2d(-36, 51.5, Math.toRadians(270));
        Pose2d driveToWhitePose3 = new Pose2d(-27, 51.5, Math.toRadians(270));
        Pose2d whiteTrussPose = new Pose2d(-35.5,-37, Math.toRadians(270));
        Pose2d whiteScoringPose = new Pose2d(-60, 24, Math.toRadians(270));
        Pose2d parkingPose = new Pose2d(-40.5, 52.75, Math.toRadians(90));

        //This action drives to the first tape line
        TrajectoryActionBuilder purpleTAction1 = drive.actionBuilder(startPose)
                .setTangent(0)
                .lineToX(-55)
                .splineTo(new Vector2d(-36, 30), Math.toRadians(270));
        Action purpleAction1 = purpleTAction1.build();

        //This action drives to the second tape line
        TrajectoryActionBuilder purpleTAction2 = drive.actionBuilder(startPose)
                .setTangent(0)
                .lineToX(-55)
                .splineTo(new Vector2d(-30, 22), Math.toRadians(270));
        Action purpleAction2 = purpleTAction2.build();

        //This action drives to the third tape line
        TrajectoryActionBuilder purpleTAction3 = drive.actionBuilder(startPose)
                .setTangent(0)
                .lineToX(-55)
                .splineTo(new Vector2d(-36, 8), Math.toRadians(270));
        Action purpleAction3 = purpleTAction3.build();

        //This action drives to the first backdrop section
        TrajectoryActionBuilder yellowScoringTAction1 = drive.actionBuilder(yellowScoringPose1)
                .strafeTo(new Vector2d(-42, 45));
        Action yellowScoringAction1 = yellowScoringTAction1.build();

        //This action drives to the second backdrop section
        TrajectoryActionBuilder yellowScoringTAction2 = drive.actionBuilder(yellowScoringPose2)
                .strafeTo(new Vector2d(-36, 45));
        Action yellowScoringAction2 = yellowScoringTAction2.build();

        //This action drives to the third backdrop section
        TrajectoryActionBuilder yellowScoringTAction3 = drive.actionBuilder(yellowScoringPose3)
                .strafeTo(new Vector2d(-36,15))
        .strafeToConstantHeading(new Vector2d(-27, 45));
        Action yellowScoringAction3 = yellowScoringTAction3.build();

        //This action OVERRIDES camera and drives to the first backdrop section
        TrajectoryActionBuilder yellowScoringOverrideTAction1 = drive.actionBuilder(yellowScoringOverridePose1)
                .strafeTo(new Vector2d(-42, 51.5));
        Action yellowScoringOverrideAction1 = yellowScoringOverrideTAction1.build();

        //This action OVERRIDES camera and drives to the second backdrop section
        TrajectoryActionBuilder yellowScoringOverrideTAction2 = drive.actionBuilder(yellowScoringOverridePose2)
                .strafeTo(new Vector2d(-36, 51.5));
        Action yellowScoringOverrideAction2 = yellowScoringOverrideTAction2.build();

        //This action OVERRIDES camera and drives to the third backdrop section
        TrajectoryActionBuilder yellowScoringOverrideTAction3 = drive.actionBuilder(yellowScoringOverridePose3)
                .strafeTo(new Vector2d(-27, 51.5));
        Action yellowScoringOverrideAction3 = yellowScoringOverrideTAction3.build();

        //This action drives to robot to the white pixel stack
        TrajectoryActionBuilder driveToWhiteTAction1 = drive.actionBuilder(driveToWhitePose1)
                .strafeToConstantHeading(new Vector2d(-42,47))
                .strafeToConstantHeading(new Vector2d(-60.5, 36))
                .strafeToConstantHeading(new Vector2d(-60.5,-36))
                .strafeToConstantHeading(new Vector2d(-35.5,-37));
        Action driveToWhiteAction1 = driveToWhiteTAction1.build();

        //This action drives to robot to the white pixel stack
        TrajectoryActionBuilder driveToWhiteTAction2 = drive.actionBuilder(driveToWhitePose2)
                .strafeToConstantHeading(new Vector2d(-36,47))
                .strafeToConstantHeading(new Vector2d(-60.5, 36))
                .strafeToConstantHeading(new Vector2d(-60.5,-36))
                .strafeToConstantHeading(new Vector2d(-35.5,-37));
                /*.strafeToConstantHeading(new Vector2d(-36,47))
                .strafeToLinearHeading(new Vector2d(-63, 36), Math.toRadians(267))//267
                .strafeToConstantHeading(new Vector2d(-63,-36))
                .strafeToLinearHeading((new Vector2d(-35,-36.5)), Math.toRadians(270));*/
        Action driveToWhiteAction2 = driveToWhiteTAction2.build();

        //This action drives to robot to the white pixel stack
        TrajectoryActionBuilder driveToWhiteTAction3 = drive.actionBuilder(driveToWhitePose3)
                .strafeToConstantHeading(new Vector2d(-28,47))
                .strafeToConstantHeading(new Vector2d(-60.5, 36))
                .strafeToConstantHeading(new Vector2d(-60.5,-36))
                .strafeToConstantHeading(new Vector2d(-35.5,-37));
                /*.strafeToConstantHeading(new Vector2d(-30,47))
                .strafeToLinearHeading(new Vector2d(-63, 36), Math.toRadians(267))
                .strafeToConstantHeading(new Vector2d(-63,-36))
                .strafeToLinearHeading(new Vector2d(-35,-36.5), Math.toRadians(270));*/
        Action driveToWhiteAction3 = driveToWhiteTAction3.build();

        //This action drives to robot to the white pixel stack
        TrajectoryActionBuilder whiteTrussTAction = drive.actionBuilder(whiteTrussPose)
                .strafeToConstantHeading(new Vector2d(-60.5,-36))
                .strafeToConstantHeading(new Vector2d(-60.5,-12))
                .strafeToConstantHeading(new Vector2d(-60.5, 12))
                .strafeToConstantHeading(new Vector2d(-60,24));
        Action whiteTrussAction = whiteTrussTAction.build();

        TrajectoryActionBuilder whiteScoringTAction = drive.actionBuilder(whiteScoringPose)
                .splineTo(new Vector2d(-40.5, 52.75), Math.toRadians(90));
        Action whiteScoringAction = whiteScoringTAction.build();

        //This action drives to robot to the parking zone
        TrajectoryActionBuilder parkingTAction = drive.actionBuilder(parkingPose)
                .strafeToConstantHeading(new Vector2d(-38,40))
        .strafeToConstantHeading((new Vector2d(-62, 50)));
        Action parkingAction = parkingTAction.build();


        //Huskylens Setup
        Deadline rateLimit = new Deadline(1, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);


        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {

            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());// this gives you the data
                telemetry.addData("location?", blocks[i].x);// this gives you just x

                //----------------------------1----------------------------\\
                if (blocks[i].x < 100 && blocks[i].id == 2 && blocks[i].y < 200) {

                    Actions.runBlocking(
                            new SequentialAction(
                                    new ParallelAction(
                                            presets.StartPos(),
                                            purpleAction1
                                    ),
                                    new SleepAction(0.2),
                                    claw.openLClaw(),
                                    new SleepAction(0.35),
                                    new ParallelAction(
                                            new ParallelAction(
                                                    presets.ScoringPos(),
                                                    yellowScoringAction1
                                            ),
                                            claw.closeLClaw()
                                    )
                            )
                    );

                    camera.alignToTag(1);

                    if(!camera.targetFound){
                        Actions.runBlocking(
                                yellowScoringOverrideAction1
                        );
                    }

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.GroundPos(),
                                                    claw.openClaws(),
                                                    claw.whiteGroundClaw()
                                            ),
                                            driveToWhiteAction1
                                    ),
                                    new SequentialAction(
                                            presets.WhiteStack(),
                                            new SleepAction(0.25),
                                            whiteTrussAction,
                                            new ParallelAction(
                                                    whiteScoringAction,
                                                    presets.WhiteScoringPos()
                                            )
                                    ),

                                    new SequentialAction(
                                            new SleepAction(.1),
                                            claw.openClaws(),
                                            new SleepAction(.25),
                                            parkingAction,
                                            presets.WhiteGroundPos()
                                    )
                            )
                    );
                    sleep(400000);
                }


                //----------------------------2----------------------------\\
                if (blocks[i].x > 100 && blocks[i].x < 200 && blocks[i].id == 2 && blocks[i].y < 200) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    new ParallelAction(
                                            presets.StartPos(),
                                            purpleAction2
                                    ),
                                    new SleepAction(0.1),
                                    claw.openLClaw(),
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.ScoringPos(),
                                                    yellowScoringAction2
                                            ),
                                            claw.closeLClaw()
                                    )
                            )
                    );

                    camera.alignToTag(2);

                    if(!camera.targetFound){
                        Actions.runBlocking(
                                yellowScoringOverrideAction2
                        );
                    }

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.GroundPos(),
                                                    claw.openClaws(),
                                                    claw.whiteGroundClaw()
                                            ),
                                            driveToWhiteAction2
                                    ),
                                    new SequentialAction(
                                            presets.WhiteStack(),
                                            new SleepAction(0.25),
                                            whiteTrussAction,
                                            new ParallelAction(
                                                    whiteScoringAction,
                                                    presets.WhiteScoringPos()
                                            )
                                    ),

                                    new SequentialAction(
                                            new SleepAction(.25),
                                            claw.openClaws(),
                                            new SleepAction(.25),
                                            parkingAction,
                                            presets.WhiteGroundPos()
                                    )
                            )
                    );
                    sleep(400000);
                }


                //----------------------------3---------------------------\\
                if (blocks[i].x > 210 && blocks[i].id == 2 && blocks[i].y < 200) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    new ParallelAction(
                                            presets.StartPos(),
                                            purpleAction3
                                    ),
                                    new SleepAction(0.1),
                                    claw.openLClaw(),
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.ScoringPos(),
                                                    yellowScoringAction3
                                            ),
                                            claw.closeLClaw()
                                    )
                            )
                    );

                    camera.alignToTag(3);

                    if(!camera.targetFound){
                        Actions.runBlocking(
                                yellowScoringOverrideAction3
                        );
                    }

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.GroundPos(),
                                                    claw.openClaws(),
                                                    claw.whiteGroundClaw()
                                            ),
                                            driveToWhiteAction3
                                    ),
                                    new SequentialAction(
                                            presets.WhiteStack(),
                                            new SleepAction(0.25),
                                            whiteTrussAction,
                                            new ParallelAction(
                                                    whiteScoringAction,
                                                    presets.WhiteScoringPos()
                                            )
                                    ),

                                    new SequentialAction(
                                            new SleepAction(.1),
                                            claw.openClaws(),
                                            new SleepAction(.25),
                                            parkingAction,
                                            presets.WhiteGroundPos()
                                    )
                            )
                    );
                    sleep(400000);
                }
            }
        }

    }


}

