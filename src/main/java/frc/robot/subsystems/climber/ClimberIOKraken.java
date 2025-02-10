package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOKraken implements ClimberIO {
        private TalonFX climberLead;
        private TalonFX climberFollow;
        private StatusSignal<Current> motorStatorCurrent;
        private StatusSignal<Angle> motorPosition;
        
        public ClimberIOKraken(){
            setup();
        }
    
        public void setup(){
            climberLead = new TalonFX(18);
            climberFollow = new TalonFX(19);    
    
            TalonFXConfiguration climberConfigs = new TalonFXConfiguration();
            climberLead.getConfigurator().apply(climberConfigs);
            climberFollow.getConfigurator().apply(climberConfigs);
    
            climberConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            climberConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
            climberConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
            climberConfigs.CurrentLimits.StatorCurrentLimit = 40;

            configurePID(ClimberConstants.PID.kPUp,ClimberConstants.PID.kIUp,ClimberConstants.PID.kDUp,ClimberConstants.PID.kFFUp);
    
            climberConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            climberConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.Setpoints.topLimit;
            climberConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            climberConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.Setpoints.bottomLimit;
    
            climberLead.getConfigurator().apply(climberConfigs);
            climberFollow.getConfigurator().apply(climberConfigs);

            climberFollow.setControl(new Follower(climberLead.getDeviceID(), false));

            motorStatorCurrent = climberLead.getStatorCurrent();
            motorPosition = climberLead.getPosition();
            BaseStatusSignal.setUpdateFrequencyForAll(50, motorPosition, motorStatorCurrent);
            climberLead.optimizeBusUtilization();
        }
    
        @Override
        public void updateInputs(ClimberIOInputs inputs) {
            inputs.kPUp = ClimberConstants.PID.kPUp;
            inputs.kIUp = ClimberConstants.PID.kIUp;
            inputs.kDUp = ClimberConstants.PID.kDUp;
            inputs.kFFUp = ClimberConstants.PID.kFFUp;
    
            inputs.kPDown = ClimberConstants.PID.kPDown;
            inputs.kIDown = ClimberConstants.PID.kIDown;
            inputs.kDDown = ClimberConstants.PID.kDDown;
            inputs.kFFDown = ClimberConstants.PID.kFFDown;

            BaseStatusSignal.refreshAll(motorStatorCurrent, motorPosition);
            // Updates all of the inputs/data points being monitored about the motor
            inputs.elevatorMotorStatorCurrent = motorStatorCurrent.getValueAsDouble();
            inputs.elevatorMotorPosition = motorPosition.getValueAsDouble();
    }

    public void stop(){
        climberLead.stopMotor();
        climberFollow.stopMotor();
    }

    @Override
    public void setPO(double PO) {
      climberLead.set(PO);
    }

    @Override
    public void setPosition(double position) {
      climberLead.setPosition(position);
    }

    @Override
    public double getPosition() {
      return motorPosition.getValueAsDouble();
    }

    @Override
    public double getVelocity() {
      return climberLead.getVelocity().getValueAsDouble();
    }
}