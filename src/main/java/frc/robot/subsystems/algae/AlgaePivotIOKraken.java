package frc.robot.subsystems.algae;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants.algaePivotConstants;

public class AlgaePivotIOKraken implements AlgaePivotIO {
        private TalonFX AlgaePivotMotor;
        private StatusSignal<Current> motorStatorCurrent;
        private StatusSignal<Angle> motorPosition;
        
        public AlgaePivotIOKraken(){
            setup();
        }
    
        public void setup(){
            AlgaePivotMotor = new TalonFX(20);
    
            TalonFXConfiguration algaePivotConfigs = new TalonFXConfiguration();
            AlgaePivotMotor.getConfigurator().apply(AlgaePivotConfigs);
    
            AlgaePivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            AlgaePivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
            AlgaePivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
            AlgaePivotConfigs.CurrentLimits.StatorCurrentLimit = 40;

            configurePID(AlgaePivotConstants.PID.kPUp,AlgaePivotConstants.PID.kIUp,AlgaePivotConstants.PID.kDUp,AlgaePivotConstants.PID.kFFUp);
    
            AlgaePivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            AlgaePivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = AlgaePivotConstants.Setpoints.topLimit;
            AlgaePivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            AlgaePivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = AlgaePivotConstants.Setpoints.bottomLimit;
    
            AlgaePivotMotor.getConfigurator().apply(AlgaePivotConfigs);

            motorStatorCurrent = AlgaePivotMotor.getStatorCurrent();
            motorPosition = AlgaePivotMotor.getPosition();
            BaseStatusSignal.setUpdateFrequencyForAll(50, motorPosition, motorStatorCurrent);
            AlgaePivotMotor.optimizeBusUtilization();
        }
    
        @Override
        public void updateInputs(AlgaePivotIOInputs inputs) {
            inputs.kPUp = AlgaePivotConstants.PID.kPUp;
            inputs.kIUp = AlgaePivotConstants.PID.kIUp;
            inputs.kDUp = AlgaePivotConstants.PID.kDUp;
            inputs.kFFUp = AlgaePivotConstants.PID.kFFUp;
    
            inputs.kPDown = AlgaePivotConstants.PID.kPDown;
            inputs.kIDown = AlgaePivotConstants.PID.kIDown;
            inputs.kDDown = AlgaePivotConstants.PID.kDDown;
            inputs.kFFDown = AlgaePivotConstants.PID.kFFDown;

            BaseStatusSignal.refreshAll(motorStatorCurrent, motorPosition);
            // Updates all of the inputs/data points being monitored about the motor
            inputs.AlgaePivotMotorStatorCurrent = motorStatorCurrent.getValueAsDouble();
            inputs.AlgaePivotMotorPosition = motorPosition.getValueAsDouble();
    }

    public void stop(){
        AlgaePivotMotor.stopMotor();
    }

    @Override
    public void setPO(double PO) {
      AlgaePivotMotor.set(PO);
    }

    @Override
    public void setPosition(double position) {
      AlgaePivotMotor.setPosition(position);
    }

    @Override
    public double getPosition() {
      return motorPosition.getValueAsDouble();
    }

    @Override
    public double getVelocity() {
      return AlgaePivotMotor.getVelocity().getValueAsDouble();
    }
}