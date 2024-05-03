package org.firstinspires.ftc.teamcode.config.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

public class PresetSubsystem {
    public static class gear {
        ClawSubsystem.claw claw = new ClawSubsystem.claw(hardwareMap);
        LiftSubsystem.lift lift = new LiftSubsystem.lift(hardwareMap);
        GearRotationSubsystem.gear gear = new GearRotationSubsystem.gear(hardwareMap);


        //------------------------------ Start Sequence ------------------------------//
        public Action StartPos() {
            return new ParallelAction(
                    ClawStartPos(),
                    LiftStartPos(),
                    GearStartPos()
            );
        }
        public Action GearStartPos() {
            return new SequentialAction(
                    gear.startGear(),
                    gear.waitForGear(),
                    gear.stopGear()
            );
        }

        public Action LiftStartPos() {
            return new SequentialAction(
                    lift.stopGear()
            );
        }

        public Action ClawStartPos() {
            return new SequentialAction(
                    claw.closeClaws(),
                    claw.groundClaw()
            );
        }

        //------------------------------ Scoring Sequence ------------------------------//

        public Action ScoringPos() {
            return new ParallelAction(
                    ClawScoringPos(),
                    LiftScoringPos(),
                    GearScoringPos()
            );
        }
        public Action GearScoringPos() {
            return new SequentialAction(
                    gear.scoringGear(),
                    gear.waitForGear(),
                    gear.stopGear()
            );
        }

        public Action LiftScoringPos() {
            return new SequentialAction(
                    lift.liftExtend_Scoring()
            );
        }

        public Action ClawScoringPos() {
            return new SequentialAction(
                    claw.scoringClaw()
            );
        }

        //------------------------------ Scoring Sequence ------------------------------//

        public Action GroundPos() {
            return new ParallelAction(
                    GearGroundPos(),
                    LiftGroundPos(),
                    ClawGroundPos()
            );
        }
        public Action GearGroundPos() {
            return new SequentialAction(
                    gear.scoringGear(),
                    gear.waitForGear(),
                    gear.stopGear()
            );
        }

        public Action LiftGroundPos() {
            return new SequentialAction(
                    lift.liftRetract_Scoring()
            );
        }

        public Action ClawGroundPos() {
            return new SequentialAction(
                    claw.groundClaw()
            );
        }



    }
}
