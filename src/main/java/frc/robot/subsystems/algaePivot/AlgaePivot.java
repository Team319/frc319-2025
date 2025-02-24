package frc.robot.subsystems.algaePivot;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class AlgaePivot extends SubsystemBase {
    private final AlgaePivotIO io;
    private final AlgaePivotIOInputsAutoLogged inputs = new AlgaePivotIOInputsAutoLogged();

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("AlgaePivot/kP");
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("AlgaePivot/kI");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("AlgaePivot/kD");

    public AlgaePivot(AlgaePivotIO io) {
        this.io = io;
        
        switch (Constants.getRobot()) {
            case COMPBOT:
            case DEVBOT:
                kP.initDefault(Constants.AlgaePivotConstants.Gains.kPUp);
                kI.initDefault(Constants.AlgaePivotConstants.Gains.kIUp);
                kD.initDefault(Constants.AlgaePivotConstants.Gains.kDUp);
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
        Logger.processInputs("/RealOutputs/AlgaePivot", inputs);
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
    
      public void configurePID(double kP, double kI, double kD) {
        io.configurePID(kP, kI, kD);
      }
    
      public double getPosition() {
        return io.getPosition();
      }
    
      public double getVelocity() {
        return io.getVelocity();
      }

      public void runPosition(double position){
        io.runPosition(position);
      }
      
    }
    


