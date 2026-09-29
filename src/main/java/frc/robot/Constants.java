package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class IntakeConstants {
    public static final int canId = 18;
    // public static final int canId = 11;
    public static final int leftCanId = 13;
  }
  public static final class IndexerConstants {
    public static final int canId = 10;
    public static final int secondCanId = 14;
    public static final int spindexerCanId = 15;
    public static final int wheelCanId = 20;
  }
  public static final class ShooterConstants {
    public static final int shooterCanId = 12;
    public static final int turntableCanId = 9;

    public static final double kMaxRotationLeft = -152.9;
    public static final double kMaxRotationRight = 188.9;
  }
  public static final class ClimbConstants {
    public static final int climbCanId = 16;
  }
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 6.8 / 6;
    public static final double kMaxSpeed = 6.8;
    public static final double kMaxAngularSpeed = 3 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(27);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(27);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));


    public static final double kFrontLeftChassisAngularOffset = Math.PI * 1.5;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final double kFrontRightChassisAngularOffset = 0;//Math.PI * 1.5;
    public static final double kBackLeftChassisAngularOffset = Math.PI;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 16;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 19) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kTurretControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 5.74;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = 3 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = (3 * Math.PI) * (3 * Math.PI);

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 2;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class LimelightConstants {
    public static final class PhysicalConstants {
      public static final double kLimelightHeightInches = 26;
      public static final double kHubAprilTagHeightInches = 44.25;
      public static final double kLimelightAngleDegrees = 7.873;
    }

    public static final class AlignConstants {
      public static final double kP_Rot = 0.0015;
      public static final double kP_Strafe = 0.0015;
      public static final double kP_Range = 0.01;
    }

    public static final class PositionConstants {
      public static final double kP_Rot = 0.01;
      public static final double kP_Strafe = 0.01;
      public static final double kP_Distance = 0.05;
    }

    public static final class AimAssistConstants {
      public static final double kP_Rot = 0.01;
    }
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }
}
