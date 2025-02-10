package frc.robot.subsystems.algaePivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants.AlgaePivotConstants;

public class AlgaePivotIOKraken implements AlgaePivotIO {
        private TalonFX algaePivotMotor;
        private StatusSignal<Current> motorStatorCurrent;
        private StatusSignal<Angle> motorPosition;
        
        public AlgaePivotIOKraken(){
            setup();
        }
    
        public void setup(){
            algaePivotMotor = new TalonFX(20);
    
            TalonFXConfiguration algaePivotConfigs = new TalonFXConfiguration();
            algaePivotMotor.getConfigurator().apply(algaePivotConfigs);
    
            algaePivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            algaePivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
            algaePivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
            algaePivotConfigs.CurrentLimits.StatorCurrentLimit = 40;

            configurePID(AlgaePivotConstants.PID.kPUp,AlgaePivotConstants.PID.kIUp,AlgaePivotConstants.PID.kDUp,AlgaePivotConstants.PID.kFFUp);
    
            algaePivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            algaePivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = AlgaePivotConstants.Setpoints.topLimit;
            algaePivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            algaePivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = AlgaePivotConstants.Setpoints.bottomLimit;
    
            algaePivotMotor.getConfigurator().apply(algaePivotConfigs);

            motorStatorCurrent = algaePivotMotor.getStatorCurrent();
            motorPosition = algaePivotMotor.getPosition();
            BaseStatusSignal.setUpdateFrequencyForAll(50, motorPosition, motorStatorCurrent);
            algaePivotMotor.optimizeBusUtilization();
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
            inputs.algaePivotMotorStatorCurrent = motorStatorCurrent.getValueAsDouble();
            inputs.algaePivotMotorPosition = motorPosition.getValueAsDouble();
    }

    public void stop(){
        algaePivotMotor.stopMotor();
    }

    @Override
    public void setPO(double PO) {
      algaePivotMotor.set(PO);
    }

    @Override
    public void setPosition(double position) {
      algaePivotMotor.setPosition(position);
    }

    @Override
    public double getPosition() {
      return motorPosition.getValueAsDouble();
    }

    @Override
    public double getVelocity() {
      return algaePivotMotor.getVelocity().getValueAsDouble();
    }
}