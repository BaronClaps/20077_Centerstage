package org.firstinspires.ftc.teamcode.config.roadrunner.driveTrajectories;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.roadrunner.MecanumDrive;

public class Blue_Close_TwoZero_Trajectories {
    /*
    public static class driveTraj {

        Pose2d beginPose = new Pose2d(-62, 12, 0);
        Pose2d scoringPose1 = new Pose2d(-42, 55, Math.toRadians(270));
        Pose2d scoringPose2 = new Pose2d(-36, 55, Math.toRadians(270));
        Pose2d scoringPose3 = new Pose2d(-30, 55, Math.toRadians(270));


        //This action drives to the first tape line
        TrajectoryActionBuilder purpleTAction1 = drive.actionBuilder(beginPose)
                .lineToX(-55)
                .splineTo(new Vector2d(-40, 29), Math.toRadians(270));

        public Action purpleAction1 = purpleTAction1.build();

        //This action drives to the second tape line
        TrajectoryActionBuilder purpleTAction2 = drive.actionBuilder(beginPose)
                .lineToX(-55)
                .splineTo(new Vector2d(-28.5, 24), Math.toRadians(270));

        public Action purpleAction2 = purpleTAction2.build();

        //This action drives to the third tape line
        TrajectoryActionBuilder purpleTAction3 = drive.actionBuilder(beginPose)
                .lineToX(-55)
                .splineTo(new Vector2d(-33.5, 10.5), Math.toRadians(270));

        public Action purpleAction3 = purpleTAction3.build();

        //This action drives to the first backdrop section


        TrajectoryActionBuilder yellowTAction1 = purpleTAction1.fresh()
                .turnTo((Math.toRadians(270)))
                .strafeTo(new Vector2d(-38, 45))
                .turnTo((Math.toRadians(270)));

        public Action yellowAction1 = yellowTAction1.build();


        //This action drives to the second backdrop section
        TrajectoryActionBuilder yellowTAction2 = purpleTAction2.fresh()
                .turnTo(Math.toRadians(270))
                .strafeTo(new Vector2d(-33, 45))
                .turnTo(Math.toRadians(270));

        public Action yellowAction2 = yellowTAction2.build();

        //This action drives to the third backdrop section
        TrajectoryActionBuilder yellowTAction3 = purpleTAction3.fresh()
                .lineToY(15)
                .turnTo((Math.toRadians(270)))
                .strafeTo(new Vector2d(-22, 45))
                .turnTo((Math.toRadians(270)));

        public Action yellowAction3 = yellowTAction3.build();


        //dfgjkhjkdfghjkhjkdfgdfghjkh fdhklfghjkl need diff poses if camera
        //This action drives to robot to the first parking zone
        TrajectoryActionBuilder parkingTAction1 = drive.actionBuilder(scoringPose1)
                .lineToY(43)
                .strafeTo((new Vector2d(-67, 50)));

        public Action parkingAction1 = parkingTAction1.build();

        //This action drives to robot to the second parking zone
        TrajectoryActionBuilder parkingTAction2 = drive.actionBuilder(scoringPose2)
                .lineToY(43)
                .strafeTo((new Vector2d(-67, 50)));

        public Action parkingAction2 = parkingTAction2.build();

        //This action drives to robot to the third parking zone
        TrajectoryActionBuilder parkingTAction3 = drive.actionBuilder(scoringPose3)
                .lineToY(43)
                .strafeTo((new Vector2d(-67, 50)));

        public Action parkingAction3 = parkingTAction3.build();

        final HardwareMap hardwareMap = null;
        public driveTraj(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
        }
    }*/
}
