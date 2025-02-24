// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.subsystems.algaePivot.AlgaePivot;
import frc.robot.subsystems.algaePivot.AlgaePivotIO;
import frc.robot.subsystems.algaePivot.AlgaePivotIOKraken;

import frc.robot.subsystems.algaeRoller.AlgaeRoller;
import frc.robot.subsystems.algaeRoller.AlgaeRollerIO;
import frc.robot.subsystems.algaeRoller.AlgaeRollerIOKraken;

import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOKraken;

import frc.robot.subsystems.coralPivot.CoralPivot;
import frc.robot.subsystems.coralPivot.CoralPivotIO;
import frc.robot.subsystems.coralPivot.CoralPivotIOKraken;
import frc.robot.subsystems.coralRoller.CoralRoller;
import frc.robot.subsystems.coralRoller.CoralRollerIO;
import frc.robot.subsystems.coralRoller.CoralRollerIOKraken;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOKraken;

/** Add your docs here. */
public class Superstructure extends SubsystemBase {

    public final Elevator elevator;  
    public final CoralPivot coralPivot;
    public final CoralRoller coralRoller;
    public final AlgaePivot algaePivot;
    public final AlgaeRoller algaeRoller;
    public final Climber climber;

    public Superstructure(){
        switch(Constants.getRobot()){
            case COMPBOT:
                
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

                break;

            case DEVBOT:
            case SIMBOT:
            default:
                // Sim robot, instantiate physics sim IO implementations
    
                elevator =
                    new Elevator(
                    new ElevatorIO() {} );
    
                algaePivot =
                    new AlgaePivot(
                    new AlgaePivotIO() {} );
    
                algaeRoller = 
                    new AlgaeRoller(
                    new AlgaeRollerIO() {} );
                
                climber =
                    new Climber(
                    new ClimberIO() {} );
                
                coralPivot =
                    new CoralPivot(
                    new CoralPivotIO() {} );  
    
                coralRoller = 
                    new CoralRoller(
                    new CoralRollerIO() {} );
                    
                break;
      }
    } // End Constructor


    // ==============================================================
    //                      ROBOT STATES
    // ==============================================================

    public enum RobotState {
        DISABLED("Disabled"),
        STARTUP_HOLD("Startup/HOLD"),
        IDLE("Idle"),
        GO_HOME("Go Home"),
        //MOVING("Elevator moving"),
        //AT_POSITION("At Position"),
        //GOTO_L1("Go To Level 1"),
        //GOTO_L2("Go To Level 2"),
        //GOTO_L3("Go To Level 3"),
        //GOTO_L4("Go To Level 4"),
        //GOTO_A2("Go To Algea on Level 2"),
        //GOTO_A3("Go To Algea on Level 3"),
        SCORE("Scoring"),
        COLLECT_FLUSH("Collecting Flush"),
        COLLECT_OBSTRUCTED("Collecting Obstructed"),
        FLOOR_COLLECT_ALGEA("Floor Collect Algea"),
        SCORE_IN_PROCESSOR("Score in Processor"),
        PREPARE_TO_CLIMB("Prepare to climb"),
        CLIMB("Climb"),
        D_OH("D'OH!");

        private final String displayName;

        RobotState(String displayName) {
            this.displayName = "["+displayName+"]";
        }

        public String getDisplayName() {
            return displayName;
        }

        @Override
        public String toString() {
            return displayName;
        }
    }

    private RobotState requestedRobotState = RobotState.IDLE;
    private RobotState lastRequestedRobotState = requestedRobotState;

    private RobotState currentState = RobotState.DISABLED;
    private RobotState lastState = currentState;

    public RobotState getCurrentState() {
        return currentState;
    }

    // Nicely ask the state machine to try and change the state
    public void requestRobotStateChange(RobotState newRobotState)
    {
        requestedRobotState = newRobotState;
    }

    /// Force the robot state to change from outside class...
    public void setCurrentState(RobotState newRobotState) {
        System.out.println("[Superstructure]: FORCE a state change");
        currentState = newRobotState;
    }

    // =================================================================

    boolean isClearFromClimber = false;
    boolean isReadyToClimb = false;
    Boolean detectedCoral = false;

    @Override
    public void periodic() {

        /* 

        // Perform Passive checks here...

        if(lastState != currentState){
            System.out.println("[Superstructure]: State changed from " + lastState.toString() + " to " + currentState.toString() );
            // Since there was a change... update the last state to this new state...
            lastState = currentState;
        }

        isClearFromClimber = ( EqualsUtil.epsilonEquals(elevator.getPosition(), ElevatorConstants.Setpoints.readyToClimb )
                                && EqualsUtil.epsilonEquals(coralPivot.getPosition() , CoralPivotConstants.Setpoints.readyToClimb) );

        isReadyToClimb = ( isClearFromClimber 
                            && EqualsUtil.epsilonEquals(climber.getPosition(),
                                                             ClimberConstants.Setpoints.readyToClimb ) ); 

        // State Machine Logic
        switch (currentState) {

            case IDLE:
            
                // Behavior when waiting to be told to do something... Zzz... Zzzzz...

                // Has a new request been made?
                if (requestedRobotState != lastRequestedRobotState ){

                    boolean isStateChangeApproved = false;
                    switch( requestedRobotState ) {
                        // Auto Accept
                        case STARTUP_HOLD:
                        case GO_HOME:
                        case SCORE:
                        case COLLECT_FLUSH:
                        case COLLECT_OBSTRUCTED:
                        case PREPARE_TO_CLIMB:
                            
                            isStateChangeApproved = true;
                            break;

                        // if there are any cases where we need to be careful... they go here.

                        // Auto Reject
                        case D_OH:
                        default:
                            isStateChangeApproved = true;
                            break;
                    }

                    if (isStateChangeApproved) { 
                        currentState = requestedRobotState; 
                        lastRequestedRobotState = requestedRobotState;  // Update last requested state to avoid duplicate processing
                    }
                
                }
                break;



            // ==========================================================================================
            //                                      "Go To" States
            // ==========================================================================================

            case GO_HOME:
                // Hold current positions of PID Controlled Subsystems, and stop roller just in case...
                elevator.runPosition(ElevatorConstants.Setpoints.bottomLimit);
                coralPivot.runPosition(CoralPivotConstants.Setpoints.home);
                coralRoller.setPO(0);

                algaePivot.runPosition(AlgaePivotConstants.Setpoints.home);
                algaeRoller.setPO(0);

                currentState = RobotState.IDLE;

                break;

            // ==========================================================================================
            //                                      Collecting States
            // ==========================================================================================

            case COLLECT_FLUSH:

                elevator.runPosition(ElevatorConstants.Setpoints.collect_flush);
                coralRoller.setPO(CoralRollerConstants.Speeds.collect);

                if(coralRoller.getStatorCurrent() >= 1.0){ // TODO = tune me
                    coralRoller.setPO(CoralRollerConstants.Speeds.stop);
                    currentState = RobotState.IDLE;
                }

                // We keep collecting until the current spike indicating a coral is in the rollers...

                break;

            case COLLECT_OBSTRUCTED:

                elevator.runPosition(ElevatorConstants.Setpoints.collect_obstructed);
                coralRoller.setPO(CoralRollerConstants.Speeds.collect);

                if(coralRoller.getStatorCurrent() >= 1.0){ // TODO = tune me
                    coralRoller.setPO(CoralRollerConstants.Speeds.stop);
                    currentState = RobotState.IDLE;
                }

                // We keep collecting until the current spike indicating a coral is in the rollers...

                break;

            // ==========================================================================================
            //                                   Algea / Processor States
            // ==========================================================================================
            case FLOOR_COLLECT_ALGEA:
                algaePivot.runPosition(AlgaePivotConstants.Setpoints.collect);
                algaeRoller.setPO(AlgaeRollerConstants.Speeds.collect);

                if(algaeRoller.getStatorCurrent() >= 1.0){
                    algaePivot.runPosition(AlgaePivotConstants.Setpoints.home);
                    algaeRoller.setPO(AlgaeRollerConstants.Speeds.stop);
                    currentState = RobotState.IDLE;
                }

                break;

            case SCORE_IN_PROCESSOR:
                // This is a perfect advocate for making a command that is scheduled to score for a time...
                //scoreInProcessor(); // ???
                break;
            // ==========================================================================================
            //                                      Climbing States
            // ==========================================================================================

            case PREPARE_TO_CLIMB:

                // Move the Elevator / Coral EndEffector out of dodge
                elevator.runPosition(ElevatorConstants.Setpoints.readyToClimb);
                coralPivot.runPosition(CoralPivotConstants.Setpoints.readyToClimb);

                if (isClearFromClimber){
                    climber.runPosition(ClimberConstants.Setpoints.topLimit);

                    currentState = RobotState.IDLE;
                }

                break;

            case CLIMB:
                if(isReadyToClimb){ climber.runPosition(ClimberConstants.Setpoints.climb); }
                else{ currentState = RobotState.D_OH; }

                // Never really leaves this state unless FORCED... this should be the end of a match. --- Tis a DEATHGRIP ---
                
                break;

            // ==========================================================================================
            //                                  Safety / Utility States
            // ==========================================================================================

            case DISABLED:
                // Stop all motors
                elevator.stop();
                coralPivot.stop();
                coralRoller.stop();
                algaePivot.stop();
                algaeRoller.stop();
                climber.stop();
                
                break;

            case STARTUP_HOLD:
                // Hold current positions of PID Controlled Subsystems
                elevator.runPosition(elevator.getPosition());
                coralPivot.runPosition(coralPivot.getPosition());
                algaePivot.runPosition(algaePivot.getPosition());

                currentState = RobotState.IDLE;

                break;

            case D_OH:
                System.out.println("[Superstructure]: D'OH! - Something went wrong. Go back to Idle");
                currentState = RobotState.IDLE;
                break;
        

            default:
                currentState = RobotState.IDLE;
                break;
                
        } // end switch(currentState)
*/
    } // end periodic()

    /* 
    private Command scoreInProcessor(){
        return Commands.sequence(Commands.deadline(new WaitCommand(1), 
                                    Commands.runOnce(()-> algaeRoller.setPO(AlgaeRollerConstants.Speeds.eject))),
                                Commands.runOnce(()->algaeRoller.stop() , algaeRoller)) ;
    }
                                */

}// end class Superstructure 
