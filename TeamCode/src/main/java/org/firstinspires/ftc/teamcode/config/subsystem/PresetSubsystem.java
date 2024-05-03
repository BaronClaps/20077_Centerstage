package org.firstinspires.ftc.teamcode.config.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

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
                    gear.stopGear(),
                    gear.resetGear()
            );
        }

        public Action LiftStartPos() {
            return new SequentialAction(
                    lift.stopLift(),
                    lift.resetLift()
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
                    lift.liftExtend_Scoring(),
                    lift.waitForLift(),
                    lift.stopLift()
            );
        }

        public Action ClawScoringPos() {
            return new SequentialAction(
                    claw.scoringClaw()
            );
        }

        //------------------------------ Ground after Scoring Sequence ------------------------------//

        public Action GroundPos() {
            return new ParallelAction(
                    GearGroundPos(),
                    LiftGroundPos(),
                    ClawGroundPos()
            );
        }
        public Action GearGroundPos() {
            return new SequentialAction(
                    gear.groundGear(),
                    gear.waitForGear(),
                    gear.stopGear()
            );
        }

        public Action LiftGroundPos() {
            return new SequentialAction(
                    lift.liftRetract_Scoring(),
                    lift.waitForLift(),
                    lift.stopLift()
            );
        }

        public Action ClawGroundPos() {
            return new SequentialAction(
                    claw.groundClaw()
            );
        }

        //------------------------------ White Stack Sequence------------------------------//
        public Action WhiteStack() {
            return new SequentialAction(
                    WhiteStackStart(),
                    new SleepAction(1),
                    WhiteStackEnd()
            );
        }

        /* White Stack Start */
        public Action WhiteStackStart() {
            return new SequentialAction(
                    GearWhiteStackStart(),
                    ClawWhiteStackStart(),
                    LiftWhiteStackStart()
            );
        }
        public Action GearWhiteStackStart() {
            return new SequentialAction(
                    gear.stopGear(),
                    gear.wheelServo_Activated()
            );
        }

        public Action LiftWhiteStackStart() {
            return new SequentialAction(
                    lift.liftExtend_Stack(),
                    lift.waitForLift(),
                    lift.stopLift()
            );
        }

        public Action ClawWhiteStackStart() {
            return new SequentialAction(
                    claw.groundClaw()
            );
        }

        /* White Stack End */
        public Action WhiteStackEnd() {
            return new SequentialAction(
                    ClawWhiteStackEnd(),
                    LiftWhiteStackEnd(),
                    new SleepAction(.25),
                    GearWhiteStackEnd()
            );
        }
        public Action GearWhiteStackEnd() {
            return new SequentialAction(
                    gear.stopGear(),
                    gear.wheelServo_Deactivated()
            );
        }

        public Action LiftWhiteStackEnd() {
            return new SequentialAction(
                    lift.liftRetract_Stack(),
                    lift.waitForLift(),
                    lift.stopLift()
            );
        }

        public Action ClawWhiteStackEnd() {
            return new SequentialAction(
                    claw.groundClaw(),
                    claw.closeLClaw()
            );
        }


    }
}
