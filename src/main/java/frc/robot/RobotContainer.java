// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.DriveCommands;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.drive.GyroIOPigeon2;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;


public class RobotContainer {

  // Subsystems
  public final Drive drive;
  public final Superstructure superstructure;
  /* 
  public final Elevator elevator;  
  public final CoralPivot coralPivot;
  public final CoralRoller coralRoller;
  public final AlgaePivot algaePivot;
  public final AlgaeRoller algaeRoller;
  public final Climber climber;
  */

  // Controller
  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser; // AdvantageKit Dependency
    
  
    public RobotContainer() {
      switch(Constants.getRobot()){
  
        case COMPBOT:
          drive =
            new Drive(
                 new GyroIOPigeon2() {},
                 new ModuleIOTalonFX(0),
                 new ModuleIOTalonFX(1),
                 new ModuleIOTalonFX(2),
                 new ModuleIOTalonFX(3)); 

          superstructure = new Superstructure();
          
          /*
          elevator =
                new Elevator(
                new ElevatorIOKraken() {});

          algaePivot =
                new AlgaePivot(
                new AlgaePivotIOKraken() {});

          algaeRoller = 
                new AlgaeRoller(
                new AlgaeRollerIOKraken() {} );
          
          climber =
                new Climber(
                new ClimberIOKraken() {});
          
          coralPivot =
                new CoralPivot(
                new CoralPivotIO() {});    // When this is connected set it to CoralPivotIOKraken() 

          coralRoller = 
                new CoralRoller(
                new CoralRollerIO() {}
                );
          */

          break;

          case DEVBOT:

          drive =
          new Drive(
               new GyroIOPigeon2() {},
               new ModuleIOTalonFX(0),
               new ModuleIOTalonFX(1),
               new ModuleIOTalonFX(2),
               new ModuleIOTalonFX(3)
               ); 

          superstructure = new Superstructure();

          break;

          case SIMBOT:
          default:
            // Sim robot, instantiate physics sim IO implementations
            drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim() );

            superstructure = new Superstructure();
                  
            break;
      }
  
      // Set up auto routines
      autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
      
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

        /*  ============================= Drive ============================= */
  
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
  
        driverController.start().whileTrue(Commands.runOnce(
            ()-> { 
              drive.resetHeading();
            }
            )
          );

        /*  ============================= Elevator ============================= */

          driverController.y().whileTrue(Commands.run(
            ()-> {
              // elevator.setPO(.05);
               superstructure.elevator.runPosition(Constants.ElevatorConstants.Setpoints.topLimit);
            }
            )
          );

          driverController.y().whileFalse(Commands.run(
            ()-> {
              superstructure.elevator.setPO(0);
            }
            )
          );

          driverController.a().whileTrue(Commands.run(
            ()-> {
              //elevator.setPO(-.05);
              superstructure.elevator.runPosition(5);
            }
            )
          );

          driverController.a().onFalse(Commands.run(
            ()-> {
             superstructure.elevator.setPO(0);
            }
            )
          );

/*  ============================= Climber ============================= */

          operatorController.y().whileTrue(Commands.run(
            ()-> {
              //climber.setPO(.5);
              superstructure.climber.runPosition(Constants.ClimberConstants.Setpoints.topLimit);
            }
            )
          );

          operatorController.y().whileFalse(Commands.run(
            ()-> {
              //climber.setPO(0);
            }
            )
          );

          operatorController.a().whileTrue(Commands.run(
            ()-> {
              //climber.setPO(-.5);
              superstructure.climber.runPosition(Constants.ClimberConstants.Setpoints.bottomLimit);
            }
            )
          );

          operatorController.a().onFalse(Commands.run(
            ()-> {
              //climber.setPO(0);
            }
            )
          );

  /*  ============================= Algae Pivot ============================= */

  operatorController.povUp().whileTrue(Commands.run(
    ()-> {
      //algaePivot.setPO(.5);
    }
    )
  );

  operatorController.povUp().whileFalse(Commands.run(
    ()-> {
      //algaePivot.setPO(0);
    }
    )
  );

  operatorController.povDown().whileTrue(Commands.run(
    ()-> {
      //algaePivot.setPO(-.5);
    }
    )
  );

  operatorController.povDown().onFalse(Commands.run(
    ()-> {
      //algaePivot.setPO(0);
    }
    )
  );

  /*  ============================= Coral Pivot ============================= */

  driverController.povUp().whileTrue(Commands.run(
    ()-> {
      //coralPivot.setPO(.5);
    }
    )
  );

  driverController.povUp().whileFalse(Commands.run(
    ()-> {
     // coralPivot.setPO(0);
    }
    )
  );

  driverController.povDown().whileTrue(Commands.run(
    ()-> {
     // coralPivot.setPO(-.5);
    }
    )
  );

  driverController.povDown().onFalse(Commands.run(
    ()-> {
      //coralPivot.setPO(0);
    }
    )
  );

  /*  ============================= Coral Rollers ============================= */

 driverController.x().whileTrue(Commands.run(
    ()-> {
      //coralRollers.setPO(.5);
    }
    )
  );

  driverController.x().whileFalse(Commands.run(
    ()-> {
      //coralRollers.setPO(0);
    }
    )
  );

  driverController.b().whileTrue(Commands.run(
    ()-> {
      //coralRollers.setPO(-.5);
    }
    )
  );

  driverController.b().onFalse(Commands.run(
    ()-> {
      //coralRollers.setPO(0);
    }
    )
  );

    /*  ============================= Algae Rollers ============================= */

    operatorController.x().whileTrue(Commands.run(
      ()-> {
        //algaeRollers.setPO(.5);
      }
      )
    );
  
    operatorController.x().whileFalse(Commands.run(
      ()-> {
        //algaeRollers.setPO(0);
      }
      )
    );
  
    operatorController.b().whileTrue(Commands.run(
      ()-> {
        //algaeRollers.setPO(-.5);
      }
      )
    );
  
    operatorController.b().onFalse(Commands.run(
      ()-> {
        //algaeRollers.setPO(0);
      }
      )
    );
  }

  
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
