package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Climber extends SubsystemBase {
    private final ClimberIO io;
    //private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();


    public Climber(ClimberIO io) {
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
        //Logger.processInputs("/RealOutputs/Climber", inputs);
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
    

