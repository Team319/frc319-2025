// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaePivotConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CoralPivotConstants;
import frc.robot.commands.AutoGoHome;
import frc.robot.commands.AutoScoreCoral;
import frc.robot.commands.CollectCoral;
import frc.robot.commands.CollectCoralObstructed;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GoHome;
import frc.robot.commands.ReadytoClimb;
import frc.robot.commands.SafelyMoveToScoringPosition;
import frc.robot.commands.ScoreCoral;
import frc.robot.subsystems.coralPivot.CoralPivotIOInputsAutoLogged;
import frc.robot.subsystems.coralPivot.CoralPivotIOKraken;
import frc.robot.subsystems.coralRoller.CoralRollerIOInputsAutoLogged;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOKraken;
import frc.robot.subsystems.coralPivot.CoralPivotIO.CoralPivotIOInputs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.drive.GyroIOPigeon2;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


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
      //Set up Named Commands in Pathplanner

      NamedCommands.registerCommand(
        "Collect",
        new CollectCoral(superstructure));

      NamedCommands.registerCommand(
        "ScoreL4",
        new SafelyMoveToScoringPosition(superstructure, 4));

        NamedCommands.registerCommand(
          "ScoreCoral",
          new AutoScoreCoral(superstructure));

          NamedCommands.registerCommand(
            "GoHome",
            new AutoGoHome(superstructure));

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

        //  ============================= Competition =============================

          // Initial Scoring commands. Requires tuned setpoints !!! - EKM 2/26

          operatorController.povUp().onTrue(new SafelyMoveToScoringPosition(superstructure, 4));

          operatorController.povRight().onTrue(new SafelyMoveToScoringPosition(superstructure, 3));

          operatorController.povDown().onTrue(new SafelyMoveToScoringPosition(superstructure, 2));


          operatorController.povLeft().onTrue(new SafelyMoveToScoringPosition(superstructure, 1)); // WARNING: This is just a start. Elevator may drop when the command finishes. Be cautious.
                                                                                                  // if it does drop, you may need to add code to the superstructure periodic to simply keep 
                                                                                                  //calling to hold some set desired 'targetPosition' in the subsystem. 
                                                                                                  // and these commands should update that 'targetPosition' variable then 
          driverController.rightBumper().whileTrue(new ScoreCoral(superstructure));
          

          //operatorController.leftTrigger().onTrue(new CollectCoral(superstructure));
          operatorController.back().onTrue(Commands.runOnce(
            ()-> {
            superstructure.coralRoller.setPO(0.2);
            }
            )
          );

          operatorController.back().onFalse(Commands.runOnce(
            ()-> {
            superstructure.coralRoller.setPO(0);
            }
            )
          );

          operatorController.leftTrigger().onTrue(new CollectCoral(superstructure));

          operatorController.leftTrigger().onFalse(new GoHome(superstructure));

          operatorController.start().onTrue(new ReadytoClimb(superstructure));

          operatorController.rightStick().onTrue(new GoHome(superstructure));


        /*  ============================= Elevator ============================= */

          operatorController.rightBumper().onTrue(Commands.runOnce(
            ()-> {
              // elevator.setPO(.05);
              superstructure.elevator.runPosition(superstructure.elevator.getPosition() + 2);  // Nudge the elevator up
            }
            )
          );



  /*  ============================= Coral Pivot ============================= */

  driverController.povUp().onTrue(Commands.runOnce(
    ()-> {
      //superstructure.coralPivot.setPO(.1);
      superstructure.coralPivot.runPosition(CoralPivotConstants.Setpoints.topLimit);
    }
    )
  );

  driverController.povUp().whileFalse(Commands.run(
    ()-> {
     //superstructure.coralPivot.setPO(0);
    }
    )
  );

  driverController.povDown().onTrue(Commands.runOnce(
    ()-> {
     //superstructure.coralPivot.setPO(-.1);
     superstructure.coralPivot.runPosition(CoralPivotConstants.Setpoints.bottomLimit/2.0);
    }
    )
  );

  driverController.povDown().onFalse(Commands.run(
    ()-> {
      //superstructure.coralPivot.setPO(0);
    }
    )
  );

  /*  ============================= Coral Rollers ============================= */

 driverController.x().whileTrue(Commands.run(
    ()-> {
      superstructure.coralRoller.setPO(.5);
    }
    )
  );

  driverController.x().whileFalse(Commands.run(
    ()-> {
      superstructure.coralRoller.setPO(0);
    }
    )
  );

  driverController.b().whileTrue(Commands.run(
    ()-> {
      superstructure.coralRoller.setPO(-.5);
    }
    )
  );

  driverController.b().onFalse(Commands.run(
    ()-> {
      superstructure.coralRoller.setPO(0);
    }
    )
  );

      /*  ============================= Algae Pivot ============================= */

      operatorController.x().onTrue(Commands.runOnce(
        ()-> {
          //superstructure.algaePivot.setPO(.1);

          superstructure.algaePivot.runPosition(AlgaePivotConstants.Setpoints.collect);
        }
        )
      );

      operatorController.x().whileFalse(Commands.run(
        ()-> {
          //superstructure.algaePivot.setPO(0);
        }
        )
      );

      operatorController.b().onTrue(Commands.runOnce(
        ()-> {
          //superstructure.algaePivot.setPO(-.1);
          superstructure.algaePivot.runPosition(AlgaePivotConstants.Setpoints.home);
        }
        )
      );

      operatorController.b().whileFalse(Commands.run(
        ()-> {
          //superstructure.algaePivot.setPO(0);
        }
        )
      );

    /*  ============================= Algae Rollers ============================= */

    // operatorController.x().whileTrue(Commands.run(
    //   ()-> {
    //     superstructure.algaeRoller.setPO(.5);
    //   }
    //   )
    // );
  
    // operatorController.x().whileFalse(Commands.run(
    //   ()-> {
    //     superstructure.algaeRoller.setPO(0);
    //   }
    //   )
    // );
  
    // operatorController.b().whileTrue(Commands.run(
    //   ()-> {
    //     superstructure.algaeRoller.setPO(-.5);
    //   }
    //   )
    // );
  
    // operatorController.b().onFalse(Commands.run(
    //   ()-> {
    //     superstructure.algaeRoller.setPO(0);
    //   }
    //   )
    // );

    /*  ============================= Climber ============================= */

    operatorController.y().onTrue(Commands.runOnce(
      ()-> {
        //superstructure.climber.setPO(.5);
        superstructure.climber.runPosition(Constants.ClimberConstants.Setpoints.climb );
      }
      )
    );

    operatorController.y().whileFalse(Commands.run(
      ()-> {
        //superstructure.climber.setPO(0);
      }
      )
    );

    operatorController.a().onTrue(Commands.runOnce(
      ()-> {
        //superstructure.climber.setPO(-.5);
        superstructure.climber.runPosition( Constants.ClimberConstants.Setpoints.readyToClimb);
      }
      )
    );

    operatorController.a().onFalse(Commands.run(
      ()-> {
        //superstructure.climber.setPO(0);
      }
      )
    );

  }

  
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
