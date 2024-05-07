package org.firstinspires.ftc.teamcode.opmode.auto;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.config.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.config.subsystem.*;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Red Close 2+0", group = "Red")
public class Red_Close_TwoZero extends LinearOpMode {

    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        ClawSubsystem claw = new ClawSubsystem(hardwareMap);
        LiftSubsystem lift = new LiftSubsystem(hardwareMap);
        GearRotationSubsystem gear = new GearRotationSubsystem(hardwareMap);
        Pose2d beginPose = new Pose2d(62, 12, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        PresetSubsystem presets = new PresetSubsystem(claw, lift, gear);
        CameraSubsystem camera = new CameraSubsystem(hardwareMap);
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        // Actions that run when Init
        claw.closeClaws();
        gear.wheelServo_Deactivated();
        camera.initAprilTag();
        camera.setManualExposure(2, 250);


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
                                            drive.RedCloseTwoZero_purpleAction1
                                    ),
                                    new SleepAction(0.1),
                                    claw.openLClaw(),
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.ScoringPos(),
                                                    drive.RedCloseTwoZero_yellowAction1),
                                            claw.closeLClaw()
                                    )
                            )
                    );

                    camera.alignToTag(4);

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            presets.GroundPos(),
                                            drive.RedCloseTwoZero_parkingAction1
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
                                            drive.RedCloseTwoZero_purpleAction2
                                    ),
                                    new SleepAction(0.1),
                                    claw.openLClaw(),
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.ScoringPos(),
                                                    drive.RedCloseTwoZero_yellowAction2),
                                            claw.closeLClaw()
                                    )
                            )
                    );

                    camera.alignToTag(5);

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            presets.GroundPos(),
                                            drive.RedCloseTwoZero_parkingAction2
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
                                            drive.RedCloseTwoZero_purpleAction3
                                    ),
                                    new SleepAction(0.1),
                                    claw.openLClaw(),
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.ScoringPos(),
                                                    drive.RedCloseTwoZero_yellowAction3),
                                            claw.closeLClaw()
                                    )
                            )
                    );

                    camera.alignToTag(6);

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            presets.GroundPos(),
                                            drive.RedCloseTwoZero_parkingAction3
                                    )
                            )
                    );
                    sleep(400000);
                }
            }
        }
    }

}

