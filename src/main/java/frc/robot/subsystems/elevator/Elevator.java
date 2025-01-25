package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLog;



public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    //private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();


    public Elevator(ElevatorIO io) {
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
        //Logger.processInputs("/RealOutputs/Elevator", inputs);
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
    

