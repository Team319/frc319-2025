// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.algaePivot.AlgaePivot;
import frc.robot.subsystems.algaePivot.AlgaePivotIO;
import frc.robot.subsystems.algaePivot.AlgaePivotIOKraken;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOKraken;
import frc.robot.subsystems.coralPivot.CoralPivot;
import frc.robot.subsystems.coralPivot.CoralPivotIO;
import frc.robot.subsystems.coralPivot.CoralPivotIOKraken;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOKraken;
import frc.robot.subsystems.drive.GyroIOPigeon2;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;


public class RobotContainer {

  // Subsystems
  public final Drive drive;
  public final Elevator elevator;
  public final AlgaePivot algaePivot;
  public final Climber climber;
  public final CoralPivot coralPivot;

  // Controller
  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

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

            elevator =
                new Elevator(
                new ElevatorIO() {});

          algaePivot =
                new AlgaePivot(
                new AlgaePivotIO() {});
          
          climber =
                new Climber(
                new ClimberIO() {});
          
          coralPivot =
                new CoralPivot(
                new CoralPivotIO() {});  
          break;
  
        case DEVBOT:
        case COMPBOT:
        default:
          drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
                // new GyroIOPigeon2() {},
                // new ModuleIOTalonFX(0),
                // new ModuleIOTalonFX(1),
                // new ModuleIOTalonFX(2),
                // new ModuleIOTalonFX(3)); TODO: Uncomment when we have drivetrain

          elevator =
                new Elevator(
                new ElevatorIOKraken() {});

          algaePivot =
                new AlgaePivot(
                new AlgaePivotIOKraken() {});
          
          climber =
                new Climber(
                new ClimberIOKraken() {});
          
          coralPivot =
                new CoralPivot(
                new CoralPivotIOKraken() {});    
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
              elevator.setPO(.5);
            }
            )
          );

          driverController.y().whileFalse(Commands.run(
            ()-> {
              elevator.setPO(0);
            }
            )
          );

          driverController.a().whileTrue(Commands.run(
            ()-> {
              elevator.setPO(-.5);
            }
            )
          );

          driverController.a().onFalse(Commands.run(
            ()-> {
              elevator.setPO(0);
            }
            )
          );

/*  ============================= Climber ============================= */

          operatorController.y().whileTrue(Commands.run(
            ()-> {
              climber.setPO(.5);
            }
            )
          );

          operatorController.y().whileFalse(Commands.run(
            ()-> {
              climber.setPO(0);
            }
            )
          );

          operatorController.a().whileTrue(Commands.run(
            ()-> {
              climber.setPO(-.5);
            }
            )
          );

          operatorController.a().onFalse(Commands.run(
            ()-> {
              climber.setPO(0);
            }
            )
          );

  /*  ============================= Algae Pivot ============================= */

  operatorController.povUp().whileTrue(Commands.run(
    ()-> {
      algaePivot.setPO(.5);
    }
    )
  );

  operatorController.povUp().whileFalse(Commands.run(
    ()-> {
      algaePivot.setPO(0);
    }
    )
  );

  operatorController.povDown().whileTrue(Commands.run(
    ()-> {
      algaePivot.setPO(-.5);
    }
    )
  );

  operatorController.povDown().onFalse(Commands.run(
    ()-> {
      algaePivot.setPO(0);
    }
    )
  );

  /*  ============================= Coral Pivot ============================= */

  driverController.povUp().whileTrue(Commands.run(
    ()-> {
      coralPivot.setPO(.5);
    }
    )
  );

  driverController.povUp().whileFalse(Commands.run(
    ()-> {
      coralPivot.setPO(0);
    }
    )
  );

  driverController.povDown().whileTrue(Commands.run(
    ()-> {
      coralPivot.setPO(-.5);
    }
    )
  );

  driverController.povDown().onFalse(Commands.run(
    ()-> {
      coralPivot.setPO(0);
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
