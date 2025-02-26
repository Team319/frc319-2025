// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.superstructure.Superstructure.RobotState;

public class Robot extends LoggedRobot {
  
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public boolean hasBeenEnabled = false;

  @Override
  public void robotInit() {
    //=============================================
    // START : Required setup for AdvantageKit Logging
    //=============================================
    
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);

    if (isReal()) {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        //new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging  
    } else if(isSimulation()){
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    }
    else{
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }
    
    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    
    //=============================================
    // END : Required setup for AdvantageKit Logging
    //=============================================

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    System.out.println(m_robotContainer.superstructure.coralRoller.getStatorCurrent());

  }

  @Override
  public void disabledInit() {
    m_robotContainer.superstructure.setCurrentState(RobotState.DISABLED);
  }
  

  @Override
  public void disabledPeriodic() {
      
   if (!hasBeenEnabled) {
    Optional<Alliance> allianceColor = DriverStation.getAlliance();
    allianceColor.ifPresent(alliance -> {
      if (alliance == Alliance.Red) {
        //m_robotContainer.drive.setHeading(0.0);
        //System.out.println("Red Alliance: Setting heading to 0 degrees.");
      } else if (alliance == Alliance.Blue) {
        //m_robotContainer.drive.setHeading(180.0);
        //System.out.println("Blue Alliance: Setting heading to 180 degrees.");
      }
    });

     if(!allianceColor.isPresent()){
       System.out.println("Alliance color is not set yet.");
     }
   }

  
  }

  @Override
  public void disabledExit() {
    hasBeenEnabled = true;

  }

  @Override
  public void autonomousInit() {
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

     if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
    }


  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
