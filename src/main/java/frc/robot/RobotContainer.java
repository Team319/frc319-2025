// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.DriveCommands;
import frc.robot.commands.autos.DynamicAutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.GyroIOPigeon2;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;


public class RobotContainer {

  // Subsystems
  public final Drive drive ;

  // Controller
  public final CommandXboxController driverController = new CommandXboxController(0);
  //public final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser; // AdvantageKit Dependency
    
    public RobotContainer() {
      switch(Constants.getRobot()){
  
        case SIMBOT:
          // Sim robot, instantiate physics sim IO implementations
          drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
          break;
  
        case DEVBOT:
        case COMPBOT:
        default:
          drive =
            new Drive(
                new GyroIOPigeon2() {},
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
          break;
      }
  
      // Add a text input from SmartDashboard
      SmartDashboard.putString("Auto Routine", "Populate me!");

      // Set up auto routines
      autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());


      autoChooser.addOption("DynamicAutoRoutine", new DynamicAutoRoutine(drive, "h1l1k1j1"));
      
      // Add Commands to the dashboard chooser
      //autoChooser.addOption(
      //    "Name on Dashboard", Commands);
  
      configureBindings();
    
    }
  
    private void configureBindings() {
      switch(Constants.getRobot()){
        case SIMBOT:
        case DEVBOT:
        case COMPBOT:
        default:
        /*  ============================= Defaults ============================= */
  
        drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -driverController.getLeftY(), // Note : This is X supplier because the field's X axis is down field long
              () -> -driverController.getLeftX(), // Note this is Y supplier because the field's Y axis is across the field 
              () -> -driverController.getRightY(), 
              () -> -driverController.getRightX(),
              () -> driverController.getLeftTriggerAxis()));
        
        break;
      }

      driverController.start().onTrue(Commands.runOnce(()-> { drive.resetHeading(); }));
        
      driverController.leftBumper().whileTrue( drive.pathfindThenFollowPath(Constants.DriveConstants.pathingConstraints,"goto_g"));
      driverController.rightBumper().onTrue( drive.pathfindThenFollowPath(Constants.DriveConstants.pathingConstraints,"goto_h"));

  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
