// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.Alert;
import frc.robot.util.Alert.*;
import edu.wpi.first.math.geometry.Translation2d;
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
