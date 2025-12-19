// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaePivotConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.AutoGoHome;
import frc.robot.commands.AutoScoreCoral;
import frc.robot.commands.CollectCoral;
import frc.robot.commands.CollectCoralObstructed;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GoHome;
import frc.robot.commands.ReadytoClimb;
import frc.robot.commands.SafelyMoveToScoringPosition;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.autos.DynamicAutoRoutine;
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

  private UsbCamera camera;
  // Subsystems
  public final Drive drive;
  public final Superstructure superstructure;
  

  // Controller
  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  //Dynamic Auto Routine Input String
  public String dynamicAutoInput = "";

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser; // AdvantageKit Dependency
    
    public RobotContainer() {
      switch(Constants.getRobot()){
  
        case COMPBOT:

          camera = CameraServer.startAutomaticCapture();
          camera.setResolution(640, 480);
          camera.setFPS(30);

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
                new CoralPivotIOKraken() {});    // When this is connected set it to CoralPivotIOKraken() 

          coralRoller = 
                new CoralRoller(
                new CoralRollerIOKraken() {}
                );

          */

          break;

        case DEVBOT:

          drive =
          new Drive(
               new GyroIO() {},
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


      autoChooser.addOption("DynamicAutoRoutine", null);
      
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

        //  ===========================================================================
        //  ============================= Driver Controls =============================
        //  ===========================================================================

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


        if (Constants.getDemoMode() == Constants.DemoMode.OFF){ // Turned off for safety at demos

          driverController.rightBumper().whileTrue(   new InstantCommand(()-> drive.pathfindToClosestRightReef().schedule() ) );

          driverController.leftBumper().whileTrue( new InstantCommand(()-> drive.pathfindToClosestLeftReef().schedule() ));
  
         // driverController.back().whileTrue( new InstantCommand(()-> drive.pathfindToProcessor().schedule()  ) );
        
        }
        


        //driverController.back().whileTrue( drive.pathFindToPose(DriveConstants.pathingConstraints, new Pose2d() ) );

          /*  ============================= Score Coral  ============================= */

          driverController.rightTrigger().whileTrue(new ScoreCoral(superstructure));

          driverController.rightTrigger().onFalse(new GoHome(superstructure));

          /*  ============================= Collect / Score Algea  ============================= */
 
          driverController.b().whileTrue(Commands.run(
            ()-> {
              superstructure.coralRoller.setPO(1.0);
            }
            )
          );
        
          driverController.b().onFalse(Commands.runOnce(
            ()-> {
        //      superstructure.algaeRoller.setPO(0);
        superstructure.coralRoller.setPO(0.0);

            }
            )
          );
        /* 
          driverController.y().whileTrue(Commands.run(
            ()-> {
              superstructure.algaeRoller.setPO(-.5);
            }
            )
          );

          driverController.y().onFalse(Commands.runOnce(
            ()-> {
              superstructure.algaeRoller.setPO(0);
            }
            )
          );
        */
          driverController.a().whileTrue(Commands.run(
            ()-> {
              superstructure.coralRoller.setPO(-0.1);
            }
            )
          );

          driverController.a().onFalse( Commands.runOnce(
            ()->{
                superstructure.coralRoller.setPO(0.0);
            }
          )
          );

        //  ===========================================================================
        //  ============================= Operator Controls ===========================
        //  ===========================================================================

        /*  ============================= Go To coral scoring positions  ============================= */
          
          operatorController.povUp().onTrue(new SafelyMoveToScoringPosition(superstructure, 4));

          operatorController.povRight().onTrue(new SafelyMoveToScoringPosition(superstructure, 3));

          operatorController.povDown().onTrue(new SafelyMoveToScoringPosition(superstructure, 2));


          operatorController.povLeft().onTrue(new SafelyMoveToScoringPosition(superstructure, 1)); // WARNING: This is just a start. Elevator may drop when the command finishes. Be cautious.
                                                                                                  // if it does drop, you may need to add code to the superstructure periodic to simply keep 
                                                                                                  //calling to hold some set desired 'targetPosition' in the subsystem. 
                                                                                                  // and these commands should update that 'targetPosition' variable then 

          operatorController.rightBumper().onTrue(Commands.runOnce(()-> {superstructure.elevator.nudgeElevatorSetpoint(1.0); superstructure.elevator.runPosition(superstructure.elevator.getPosition()+superstructure.elevator.getElevatorNudge());}));

          operatorController.leftBumper().onTrue(Commands.runOnce(()-> {superstructure.elevator.nudgeElevatorSetpoint(-1.0); superstructure.elevator.runPosition(superstructure.elevator.getPosition()+superstructure.elevator.getElevatorNudge());  } ));


          /*  ============================= Collect Coral ============================= */

          operatorController.leftTrigger().onTrue(new CollectCoral(superstructure));

          operatorController.rightTrigger().onTrue(new CollectCoralObstructed(superstructure));

        /*  ============================= Climbing ============================= */

          operatorController.start().onTrue(new ReadytoClimb(superstructure));

          operatorController.a().onTrue(Commands.runOnce(()-> {superstructure.climber.runPosition(ClimberConstants.Setpoints.climb); superstructure.climber.setClimbMode(true);} ));
        //  operatorController.a().onTrue(Commands.runOnce(()-> {superstructure.climber.setPO(-.1);} ));
        //  operatorController.a().onFalse(Commands.runOnce(()-> {superstructure.climber.setPO(0);} ));

          operatorController.y().onTrue(Commands.runOnce(()-> {/*superstructure.climber.runPosition(ClimberConstants.Setpoints.climb);*/superstructure.climber.setClimbMode(false);} ));
          operatorController.x().onTrue(Commands.runOnce(()-> {superstructure.climber.setClimberIntakePO(-0.25);} ));
          operatorController.x().onFalse(Commands.runOnce(()-> {superstructure.climber.setClimberIntakePO(0);} ));
         // operatorController.y().onFalse(Commands.runOnce(()-> {superstructure.climber.setPO(0.0);} ));

          operatorController.rightStick().onTrue(new GoHome(superstructure));
 

        


        /*  ============================= Elevator ============================= */

          // operatorController.rightBumper().whileTrue(Commands.run(
          //   ()-> {
          //     // elevator.setPO(.05);
          //     superstructure.elevator.runPosition(superstructure.elevator.getPosition() + 2);  // Nudge the elevator up
          //   }
          //   )
          // );


          // operatorController.leftBumper().whileTrue(Commands.run(
          //   ()-> {
          //     // elevator.setPO(.05);
          //     superstructure.elevator.runPosition(superstructure.elevator.getPosition() - 2);  // Nudge the elevator up
          //   }
          //   )
          // );
  /*  ============================= Coral Pivot ============================= */

  // driverController.povUp().onTrue(Comleftmands.runOnce(
  //   ()-> {
  //     //superstructure.coralPivot.setPO(.1);
  //     superstructure.coralPivot.runPosition(CoralPivotConstants.Setpoints.topLimit);
  //   }
  //   )
  // );

  // driverController.povUp().whileFalse(Commands.run(
  //   ()-> {
  //    //superstructure.coralPivot.setPO(0);
  //   }
  //   )
  // );

  // driverController.povDown().onTrue(Commands.runOnce(
  //   ()-> {
  //    //superstructure.coralPivot.setPO(-.1);
  //    superstructure.coralPivot.runPosition(CoralPivotConstants.Setpoints.bottomLimit/2.0);
  //   }
  //   )
  // );

  // driverController.povDown().onFalse(Commands.run(
  //   ()-> {
  //     //superstructure.coralPivot.setPO(0);
  //   }
  //   )
  // );

  /*  ============================= Coral Rollers ============================= */

//  driverController.x().whileTrue(Commands.run(
//     ()-> {
//       superstructure.coralRoller.setPO(.5);
//     }
//     )
//   );

//   driverController.x().whileFalse(Commands.run(
//     ()-> {
//       superstructure.coralRoller.setPO(0);
//     }
//     )
//   );

//   driverController.b().whileTrue(Commands.run(
//     ()-> {
//       superstructure.coralRoller.setPO(-.5);
//     }
//     )
//   );

//   driverController.b().onFalse(Commands.run(
//     ()-> {
//       superstructure.coralRoller.setPO(0);
//     }
//     )
//   );

      /*  ============================= Algae Pivot ============================= */

      // operatorController.x().onTrue(Commands.runOnce(
      //   ()-> {
      //     //superstructure.algaePivot.setPO(.3);

      //     superstructure.algaePivot.runPosition(AlgaePivotConstants.Setpoints.collect);
      //   }
      //   )
      // );
/* 
      operatorController.x().onFalse(Commands.run(
        ()-> {
          superstructure.algaePivot.setPO(0);
        }
        )
      );
*/
      // operatorController.b().onTrue(Commands.runOnce(
      //   ()-> {
      //     //superstructure.algaePivot.setPO(-.3);
      //     superstructure.algaePivot.runPosition(AlgaePivotConstants.Setpoints.home);
      //   }
      //   )
      // );
/* 
      operatorController.b().onFalse(Commands.run(
        ()-> {
          superstructure.algaePivot.setPO(0);
        }
        )
      );
*/

    }
  public Command getAutonomousCommand() {
    
    if(autoChooser.get() == null){
      return new DynamicAutoRoutine(drive,superstructure); // The command needs to be created at runtime so that the instruction string is populated from the dashboard
    }
    else{
      return autoChooser.get();
    }
    
    


  }
}