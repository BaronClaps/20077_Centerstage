package org.firstinspires.ftc.teamcode.config.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class ClawSubsystem {


    public static class claw {
        private Servo pivot = null;
        private Servo clawL = null;
        private Servo clawR = null;

        double closedL = 0.33;
        double closedR = 0.37;
        double openL = 0.42;
        double openR = 0.28;
        double groundClaw = 0.815;
        double scoringClaw = 0.23;

        public claw(HardwareMap hardwareMap) {
            pivot = hardwareMap.get(Servo.class, "pivot");
            clawL = hardwareMap.get(Servo.class, "clawL");
            clawR = hardwareMap.get(Servo.class, "clawR");
        }





        //------------------------------Close Claws------------------------------//
        public class closeLClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawL.setPosition(closedL);
                return false;
            }
        }
        public Action closeLClaw() {
            return new closeLClaw();
        }
        public class closeRClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawR.setPosition(closedR);
                return false;
            }
        }
        public Action closeRClaw() {
            return new closeRClaw();
        }
        public class closeClaws implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawL.setPosition(closedL);
                clawR.setPosition(closedR);
                return false;
            }
        }
        public Action closeClaws() {
            return new closeClaws();
        }
        //------------------------------Open Claws------------------------------//
        public class openLClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawL.setPosition(openL);
                return false;
            }
        }
        public Action openLClaw() {
            return new openLClaw();
        }

        public class openRClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawR.setPosition(openR);
                return false;
            }
        }
        public Action openRClaw() {
            return new openRClaw();
        }

        public class openClaws implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawL.setPosition(openL);
                clawR.setPosition(openR);
                return false;
            }
        }
        public Action openClaws() {
            return new openClaws();
        }

        //------------------------------Claw Rotate------------------------------//

        public class groundClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                pivot.setPosition(groundClaw);
                return false;
            }
        }
        public Action groundClaw() {
            return new groundClaw();
        }

        public class scoringClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                pivot.setPosition(scoringClaw);
                return false;
            }
        }
        public Action scoringClaw() {
            return new scoringClaw();
        }
    }
}
