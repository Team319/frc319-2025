// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants;
import frc.robot.Constants.HeadingTargets;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TargetLocations;
import frc.robot.Constants.DriveConstants;


import frc.robot.subsystems.limelight.Limelight;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.PolarCoordinate;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {


  // Motor configuration

  // PathPlanner configuration
  public static final RobotConfig ppConfig =
      new RobotConfig(
          DriveConstants.robotMassKg,
          DriveConstants.robotMOI,
          new ModuleConfig(
            DriveConstants.wheelRadiusMeters,
            DriveConstants.MAX_LINEAR_SPEED,
            DriveConstants.wheelCOF,
              DCMotor.getKrakenX60(1)
                  .withReduction(DriveConstants.motorReduction),
                  DriveConstants.currentLimit,
              1),
              DriveConstants.TRACK_WIDTH_X);

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; //  FL, FR, BL, BR
  private final SysIdRoutine sysId;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private double rawGyroVelocityRadPerSec = 0.0;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  
  private HeadingTargets headingTarget = HeadingTargets.NO_TARGET;
  private static final PIDController headingPID = new PIDController(0.4, 0.001 , 0.03); // originally 0.55, 0.0, 0.0

  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private boolean doRejectVisionUpdate = true;
	
  @AutoLogOutput(key = "/RealOutputs/Drive/headingSetpoint")
  private double headingSetpoint = 0.0;

  @AutoLogOutput(key = "/RealOutputs/Drive/headingLocked")
  private boolean headingLocked = false;

  private boolean updatePoseUsingVision = false;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;

    headingPID.enableContinuousInput(-Math.PI, Math.PI);
    headingPID.setTolerance(0.1, 0.1);

    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });

    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
		
    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("/RealOutputs/Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
  }

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("/RealOutputs/Drive/Gyro", gyroInputs);

    Logger.recordOutput("/RealOutputs/Drive/DistanceToAllianceSpeaker", getDistanceToAllianceSpeaker());

    switch (Constants.getRobot()) {
      case SIMBOT:
      case DEVBOT:
      case COMPBOT:
        // if Swerve, use and update the modules
        for (var module : modules) {
          module.periodic();
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
          for (var module : modules) {
            module.stop();
          }
        }

        // Log measured states
        Logger.recordOutput("SwerveStates/Measured", getModuleStates());

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
          Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
          Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        SwerveModulePosition[] modulePositions = getModulePositions();
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
          moduleDeltas[moduleIndex] =
              new SwerveModulePosition(
                  modulePositions[moduleIndex].distanceMeters
                      - lastModulePositions[moduleIndex].distanceMeters,
                  modulePositions[moduleIndex].angle);
          lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
        }
        // The twist represents the motion of the robot since the last
        // loop cycle in x, y, and theta based only on the modules,
        // without the gyro. The gyro is always disconnected in simulation.
        if (gyroInputs.connected) {
          // If the gyro is connected, replace the theta component of the twist
          // with the change in angle since the last loop cycle.
          rawGyroRotation = gyroInputs.yawPosition;
          rawGyroVelocityRadPerSec = gyroInputs.yawVelocityRadPerSec;
        } else {
          // Apply the twist (change since last loop cycle) to the current pose
          Twist2d twist = kinematics.toTwist2d(moduleDeltas);
          rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
          rawGyroVelocityRadPerSec = 0.0;
        }

        poseEstimator.update(rawGyroRotation, modulePositions);
        Logger.recordOutput("Odometry/Robot", getPose());
 
         
        if(Limelight.isValidTargetSeen(LimelightConstants.Device.REEF) /*&& DriverStation.isTeleop()*/ )
        {
          double [] poseBuf = Limelight.getBotPose(LimelightConstants.Device.REEF);
          Pose3d visionPose = new Pose3d(
                                new Translation3d(poseBuf[0],poseBuf[1],poseBuf[2]), 
                                new Rotation3d(Units.degreesToRadians(poseBuf[3]), Units.degreesToRadians(poseBuf[4]),Units.degreesToRadians(poseBuf[5]))
                              );
          Logger.recordOutput("Odometry/VisionPose", visionPose.toPose2d());
 
          double poseDifference = poseEstimator.getEstimatedPosition().getTranslation().getDistance(visionPose.toPose2d().getTranslation());
          Logger.recordOutput("/RealOutputs/Drive/poseDifference", poseDifference);

          LimelightHelpers.SetRobotOrientation("limelight-reef", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
          LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-reef");
          
          if(Math.abs(rawGyroVelocityRadPerSec) > Units.degreesToRadians(720) ) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
          {
            doRejectVisionUpdate = true;
          }
          if(mt2.tagCount == 0)
          {
            doRejectVisionUpdate = true;
          }
          if(!doRejectVisionUpdate)
          {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            poseEstimator.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
          }
          
        }

        break; // End of Swerve logic
    
      default:
        // Do nothing
        break;
    }
    
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns the average drive velocity in radians/sec. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocitoes) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

    /** Resets the current odometry pose. */
  public void setPose(Pose2d pose, Rotation2d newGyroRotation) {
    poseEstimator.resetPosition(newGyroRotation, getModulePositions(), pose);
  }

  public double snapToHeading(DoubleSupplier x, DoubleSupplier y) {
    
    // ===================  Thank you 1806 for the help ! =======================
    double[] rightJoyPolarCoordinate = PolarCoordinate.toPolarCoordinate(x,y);
    double r = rightJoyPolarCoordinate[0];
    double theta = rightJoyPolarCoordinate[1];
    
    if(r < 0.8){
        theta = getRotation().getRadians();
    }
    else{ // Valid Driver input
      this.headingTarget = HeadingTargets.NO_TARGET;
    }

    theta /= (Math.PI / 4);
    theta = Math.round(theta) * (Math.PI / 4);
    return headingPID.calculate(getRotation().getRadians(), theta);
  }

  public double snapToHeading() {
    return headingPID.calculate(getRotation().getRadians(), headingSetpoint);
  }

  public void setHeadingSetpoint(double headingRadians) {
    lockHeading();
    headingTarget = HeadingTargets.NO_TARGET;

    Rotation2d heading = Rotation2d.fromRadians(headingRadians);

    //if (DriverStation.getAlliance().get() == Alliance.Red){
    //  this.headingSetpoint = heading.rotateBy(Rotation2d.fromRadians(Math.PI)).getRadians();
    //}
    this.headingSetpoint = headingRadians;
  }

  public Translation2d getCurrentTargetLocation(){
    Translation2d retVal = TargetLocations.ORIGIN;

    if ( DriverStation.getAlliance().isPresent()){
      switch (this.headingTarget) {
        case SPEAKER:
          switch (DriverStation.getAlliance().get()) {
            case Red:
              retVal = TargetLocations.RED_SPEAKER;
              break;
          
            default: // Blue
              retVal = TargetLocations.BLUE_SPEAKER;
              break;
          }
          break;// Escape Speaker Case

        case SOURCE:
          switch (DriverStation.getAlliance().get()) {
            case Red:
              retVal = TargetLocations.RED_SOURCE;
              break;
          
            default: // Blue
              retVal = TargetLocations.BLUE_SOURCE;
              break;
          }
          break; // Escape Source Case
      
        default:
          retVal = TargetLocations.ORIGIN;
          break; // Escape Default Case
      }
  }
    return retVal;
  }

  public void setHeadingTarget(HeadingTargets target){
    this.headingTarget = target;
    lockHeading();
  }

  public HeadingTargets getHeadingTarget(){
    return this.headingTarget ;
  }

  public double snapToTarget() {
    // ===================  Thank you 4481 for the help ! =======================
    double theta = 0.0;
    // Target - Robot 

   boolean isTargetVisible = Limelight.isValidTargetSeen(LimelightConstants.Device.REEF);

    if(false/*isTargetVisible*/){
      //System.out.println("Target Visible, use limelight data to automatically control heading");
      theta = Limelight.getHorizontalOffset(LimelightConstants.Device.REEF);

      return headingPID.calculate(theta, 0.0); // try and make the Horizontal Offset 0, meaning the target is centered
    }
    else{
      //System.out.println("Target Not Visible, using odometry and pose for best guess");
      
      //System.out.println("Robot x:" + getPose().getTranslation().getX() + "Robot y:" + getPose().getTranslation().getY()  );
      Translation2d difference = getCurrentTargetLocation().minus(getPose().getTranslation());
      theta = difference.rotateBy(Rotation2d.fromRadians(Math.PI)).getAngle().getRadians();
    }
    return headingPID.calculate(getRotation().getRadians(), theta);
  }

  //public Optional<Rotation2d> getRotationTargetOverride(){ //was private
    
    //NOTE : Returned value must be a field relative angle

   /*  if (this.updatePoseUsingVision){
      // this expects the limelight pipeline is only filtering for speaker tags (be sure to filter both april tags for both alliances on the same speaker pipeline)
      if(Limelight.getNumTargets(LimelightConstants.Device.REEF) >= 2){ 

        System.out.println("Override Heading!!");

        //Method 1 : Use Limelight
        //double theta = Limelight.getHorizontalOffset(LimelightConstants.Device.REEF);
        //return Optional.of(Rotation2d.fromDegrees(theta));

        //Method 2 : Use Pose
        Translation2d difference = getCurrentTargetLocation().minus(getPose().getTranslation());
        double theta = difference.rotateBy(Rotation2d.fromRadians(Math.PI)).getAngle().getRadians();
        return Optional.of(Rotation2d.fromRadians(theta));
      }
      else{ // i don't see both tags...
        return Optional.empty();
      }
    }
    else // heading is unlocked
    {
      return Optional.empty();
    }
    

   */ 
  //}

  public double getAngleToCurrentTarget(){
    return getCurrentTargetLocation().minus(getPose().getTranslation()).getAngle().getRadians();
  }

  public double getAngleToTarget(Translation2d target){
    return target.minus(getPose().getTranslation()).getAngle().getRadians();
  }
 
  public double getDistanceToCurrentTarget(){
    return getCurrentTargetLocation().getDistance(getPose().getTranslation());
  }
  
  public double getDistanceToTarget(Translation2d target){
    return target.getDistance(getPose().getTranslation());
  }

  public double getDistanceToAllianceSpeaker(){
    Translation2d allianceSpeaker = TargetLocations.BLUE_SPEAKER;
    if ( DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
      allianceSpeaker = TargetLocations.RED_SPEAKER;
    }
    return getDistanceToTarget(allianceSpeaker);
  }

  public boolean isHeadingLocked() {
    return headingLocked;
  }

  public void lockHeading() {
    this.headingLocked = true;
  }

  public void unlockHeading() {
    this.headingLocked = false;
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return DriveConstants.MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0)
    };
  }

  public void resetHeading(){
    gyroIO.reset();
  }

  public void setUpdatePoseWithVision(boolean input){
    this.updatePoseUsingVision = input;
  }

// ========================= Empty case / No Drivetrain =========================
public Drive(GyroIO gyroIO){
  this.gyroIO = gyroIO;
  // Configure SysId
    sysId = null;
  }
}
