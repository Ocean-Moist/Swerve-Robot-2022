package frc.robot;


public final class Constants {
    public static final class DriveConstants {
        public static final int frontLeftDrivePort = 8;
        public static final int frontLeftTurnPort = 11;
        public static final int frontRightDrivePort = 2;
        public static final int frontRightTurnPort = 6;
        public static final int backLeftDrivePort = 5;
        public static final int backLeftTurnPort = 3;
        public static final int backRightDrivePort = 4;
        public static final int backRightTurnPort = 7;

        public static final int frontLeftTurnEncoderPort = 3;
        public static final int frontRightTurnEncoderPort = 2;
        public static final int backLeftTurnEncoderPort = 1;
        public static final int backRightTurnEncoderPort = 4;

        public static final double speedScale = 1.0;

        public static final double minimumDrivePower = 0.0;
        public static final double minimumTurnPower = 0.0;
        public static final double maximumDrivePower = 1.0;
        public static final double maximumTurnPower = 1.0;
        public static boolean kGyroReversed = false;
    }

    public static final class AutonomousConstants {

    }

    public static final class OIConstants {
        public static final int xboxControllerPort = 0;
        public static final int fightStickPort = 1;
    }

    public static final class MechanismConstants {

    }

    public static final class PneumaticsConstants {

    }
}
