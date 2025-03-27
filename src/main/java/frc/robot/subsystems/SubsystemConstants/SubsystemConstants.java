package frc.robot.subsystems.SubsystemConstants;

import frc.robot.Networking.UDPServer.Double3;

public class SubsystemConstants {
    public class ElevatorConstants{
        public static final double L1 = 0.05;
        public static final double L2 = 0.05;
        public static final double L3 = 0.05;
        public static final double L4 = 5.3;
        public static final double AlgaeReef1 = 1;
        public static final double AlgaeReef2 = 2;
        public static final double AlgaeBarge = 5.3;
        public static final double CoralStation = 1;
        public static final double Resting = 0.3;
    }
    public class ArmConstants{
        public static final double L1 = 0.46;
        public static final double L2 = 0.46;
        public static final double L3 = 0.6;
        public static final double L4 = 0.6;
        public static final double AlgaeReef1 = 0.33;
        public static final double AlgaeReef2 = 0.33;
        public static final double AlgaeBarge = 0.33;
        public static final double CoralStation = 0.05;
        public static final double Resting = 0.2;
    }

    public class ScoringPoses {
        // the number after the reef is the aprilTag number closest to the scoring position
        // ex: redReef10 = AprilTag 10
        public static final Double3 redReef10 = new Double3(12.227306 - 1 /*subtracting a little from the OG */,4.0259,180);
        public static final Double3 redReef9 = new Double3(12.643358 - 0.5,4.745482 + 0.5,120);
        public static final Double3 redReef8 = new Double3(13.474446 + 0.5,4.745482 + 0.5,60);
        public static final Double3 redReef7 = new Double3(13.890498 + 1,4.0259,0);
        public static final Double3 redReef6 = new Double3(13.474446 + 0.5,3.306318 - 0.5,300);
        public static final Double3 redReef11 = new Double3(12.643358f - 0.5,3.306318f - 0.5,240);

        public static final Double3[] redReefScoringPoses = {redReef10,redReef9,redReef8,redReef7,redReef6,redReef11};
    }
}
