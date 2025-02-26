package frc.robot.subsystems.coralRoller;

import java.lang.System.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class CoralRoller extends SubsystemBase {
    private final CoralRollerIO io;
    private final CoralRollerIOInputsAutoLogged inputs = new CoralRollerIOInputsAutoLogged();


    public CoralRoller(CoralRollerIO io) {
        this.io = io;
        
        switch (Constants.getRobot()) {
            case COMPBOT:
            case DEVBOT:
                break;
            case SIMBOT:
                break;
            default:
                break;
        } 
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        //Logger.processInputs("/RealOutputs/CoralRoller", inputs);
    }

    public void stop() {
        io.stop();
      }
    
      public void setPO(double PO) {
        io.setPO(PO);
      }
    
      public void setPosition(double position) {
        io.setPosition(position);
      }
    
      public void setVoltage(double voltage) {
        io.setVoltage(voltage);
      }
    
      public double getPosition() {
        return io.getPosition();
      }
    
      public double getVelocity() {
        return io.getVelocity();
      }

      public double getStatorCurrent(){
        return io.getStatorCurrent();
      }
      
    }
    

