package frc.robot.subsystems.algaeRoller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants.AlgaeRollerConstants;

public class AlgaeRollerIOKraken implements AlgaeRollerIO {
        private TalonFX algaeRollerMotor;
        private StatusSignal<Current> motorStatorCurrent;
        private StatusSignal<Angle> motorPosition;
        
        public AlgaeRollerIOKraken(){
            setup();
        }
    
        public void setup(){
            algaeRollerMotor = new TalonFX(21);
    
            TalonFXConfiguration algaeRollerConfigs = new TalonFXConfiguration();
            algaeRollerMotor.getConfigurator().apply(algaeRollerConfigs);
    
            algaeRollerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            algaeRollerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
            algaeRollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
            algaeRollerConfigs.CurrentLimits.StatorCurrentLimit = 40;

            configurePID(AlgaeRollerConstants.PID.kPUp,AlgaeRollerConstants.PID.kIUp,AlgaeRollerConstants.PID.kDUp,AlgaeRollerConstants.PID.kFFUp);
    
            algaeRollerConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            algaeRollerConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = AlgaeRollerConstants.Setpoints.topLimit;
            algaeRollerConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            algaeRollerConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = AlgaeRollerConstants.Setpoints.bottomLimit;
    
            algaeRollerMotor.getConfigurator().apply(algaeRollerConfigs);

            motorStatorCurrent = algaeRollerMotor.getStatorCurrent();
            motorPosition = algaeRollerMotor.getPosition();
            BaseStatusSignal.setUpdateFrequencyForAll(50, motorPosition, motorStatorCurrent);
            algaeRollerMotor.optimizeBusUtilization();
        }
    
        @Override
        public void updateInputs(AlgaeRollerIOInputs inputs) {
            inputs.kPUp = AlgaeRollerConstants.PID.kPUp;
            inputs.kIUp = AlgaeRollerConstants.PID.kIUp;
            inputs.kDUp = AlgaeRollerConstants.PID.kDUp;
            inputs.kFFUp = AlgaeRollerConstants.PID.kFFUp;
    
            inputs.kPDown = AlgaeRollerConstants.PID.kPDown;
            inputs.kIDown = AlgaeRollerConstants.PID.kIDown;
            inputs.kDDown = AlgaeRollerConstants.PID.kDDown;
            inputs.kFFDown = AlgaeRollerConstants.PID.kFFDown;

            BaseStatusSignal.refreshAll(motorStatorCurrent, motorPosition);
            // Updates all of the inputs/data points being monitored about the motor
            inputs.algaeRollerMotorStatorCurrent = motorStatorCurrent.getValueAsDouble();
            inputs.algaeRollerMotorPosition = motorPosition.getValueAsDouble();
    }

    public void stop(){
        algaeRollerMotor.stopMotor();
    }

    @Override
    public void setPO(double PO) {
      algaeRollerMotor.set(PO);
    }

    @Override
    public void setPosition(double position) {
      algaeRollerMotor.setPosition(position);
    }

    @Override
    public double getPosition() {
      return motorPosition.getValueAsDouble();
    }

    @Override
    public double getVelocity() {
      return algaeRollerMotor.getVelocity().getValueAsDouble();
    }
}