// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.Alert;
import frc.robot.util.Alert.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/** Add your docs here. */
public class Constants {
  public static final double loopPeriodSecs = 0.02;
  private static RobotType robotType = RobotType.COMPBOT;
  public static final boolean tuningMode = false;

  public static RobotType getRobot() {
    if ( RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
          .set(true);
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }

  public static Mode getMode() {
    return switch (robotType) {
      case DEVBOT, COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    SIMBOT,
    DEVBOT,
    COMPBOT
  }

  public static class DriveConstants{
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(17.3);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(22.0);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(22.0);
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
    public static final int currentLimit = 40; // TODO consider setting this to 60
    public static final double wheelRadiusMeters = Units.inchesToMeters(3.875); // Black nitrile : 3 7/8 with full tread = 3.875 inches
    public static final double motorReduction = 6.122; // Check me - EKM 11/11
    public static final double robotMassKg = 74.088; // TODO
    public static final double robotMOI = 6.883; // TODO
    public static final double wheelCOF = 1.2; // TODO
    public static final double DRIVE_GEAR_RATIO = 6.122; //(50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0); // L3
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
    public static final boolean isTurnMotorInverted = true;
  }
  public static enum HeadingTargets{
    NO_TARGET,
    SPEAKER,
    SOURCE
  }

  public static class TargetLocations{
    public static Translation2d ORIGIN = new Translation2d();
    public static Translation2d RED_SPEAKER = new Translation2d(16.45,5.3);
    public static Translation2d BLUE_SPEAKER = new Translation2d(0.0,5.3);
    public static Translation2d RED_SOURCE = new Translation2d(0.0,-0.5);
    public static Translation2d BLUE_SOURCE = new Translation2d(16.15,-0.5);
  }

  public static class LimelightConstants{
    public static enum Device{
      REEF,
      CORAL_STATION
    }
  }

}
