package frc.robot.subsystems.algaeRoller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class AlgaeRoller extends SubsystemBase {
    private final AlgaeRollerIO io;
    //private final AlgaeRollerIOInputsAutoLogged inputs = new AlgaeRollerIOInputsAutoLogged();


    public AlgaeRoller(AlgaeRollerIO io) {
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
        //io.updateInputs(inputs);
        //Logger.processInputs("/RealOutputs/=AlgaePivot", inputs);
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
    
      public void configurePID(double kP, double kI, double kD, double kFF) {
        io.configurePID(kP, kI, kD, kFF);
      }
    
      public double getPosition() {
        return io.getPosition();
      }
    
      public double getVelocity() {
        return io.getVelocity();
      }
    }
    


