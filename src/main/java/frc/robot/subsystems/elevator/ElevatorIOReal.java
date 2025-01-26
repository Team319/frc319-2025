package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {
        private TalonFX elevatorLead;
        private TalonFX elevatorFollow;
        
        public ElevatorIOReal(){
            setup();
            //setFollow();
        }
    
        public void setup(){
            elevatorLead = new TalonFX(13);
            elevatorFollow = new TalonFX(14);
    
            final double positionTargetSetpoint; //idk if this is right
    
    
            TalonFXConfiguration elevatorConfigs = new TalonFXConfiguration();
            elevatorLead.getConfigurator().apply(elevatorConfigs);
            elevatorFollow.getConfigurator().apply(elevatorConfigs);
    
            elevatorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            elevatorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
            elevatorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
            elevatorConfigs.CurrentLimits.StatorCurrentLimit = 40;

            //elevatorConfigs.Slot0.kV = ElevatorConstants.kFF;
    
            elevatorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            elevatorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.Setpoints.topLimit;
            elevatorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            elevatorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.Setpoints.bottomLimit;
    
            elevatorLead.getConfigurator().apply(elevatorConfigs);
            elevatorFollow.getConfigurator().apply(elevatorConfigs);
        }
    
       /*  public void setFollow(){
            elevatorFollow.follow(elevatorLead);
        }*/
    
        @Override
        public void updateInputs(ElevatorIOInputs inputs) {
            inputs.kPUp = ElevatorConstants.PID.kPUp;
            inputs.kIUp = ElevatorConstants.PID.kIUp;
            inputs.kDUp = ElevatorConstants.PID.kDUp;
            inputs.kFFUp = ElevatorConstants.PID.kFFUp;
    
            inputs.kPDown = ElevatorConstants.PID.kPDown;
            inputs.kIDown = ElevatorConstants.PID.kIDown;
            inputs.kDDown = ElevatorConstants.PID.kDDown;
            inputs.kFFDown = ElevatorConstants.PID.kFFDown;
    
            //inputs.targetPosition = this.positionTargetSetpoint;
    }

    public void stop(){
        elevatorLead.stopMotor();
        elevatorFollow.stopMotor();
    }
}