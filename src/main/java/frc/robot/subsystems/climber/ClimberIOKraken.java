package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOKraken implements ClimberIO {
        private TalonFX climberLead;
        private TalonFX climberFollow;
        private TalonFX climberIntake;
        private StatusSignal<Current> motorStatorCurrent;
        private StatusSignal<Angle> motorPosition;

        private final PositionVoltage positionVoltage = new PositionVoltage(0.0);
        private Slot0Configs slot0Configs = new Slot0Configs();

        TalonFXConfiguration climberConfigs = new TalonFXConfiguration();
        TalonFXConfiguration climberConfigsLead = new TalonFXConfiguration();
        
        public ClimberIOKraken(){
            setup();
        }
    
        public void setup(){
            climberLead = new TalonFX(18);
            climberFollow = new TalonFX(19);  
            climberIntake = new TalonFX(22);

            TalonFXConfiguration climberIntakeConfigs = new TalonFXConfiguration();
            climberIntake.getConfigurator().apply(climberIntakeConfigs);
  
            climberFollow.setControl(new Follower(climberLead.getDeviceID(), true));
    
            climberConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            climberConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            climberConfigsLead.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            climberConfigsLead.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    
            configurePID(ClimberConstants.Gains.kPUp, ClimberConstants.Gains.kIUp, ClimberConstants.Gains.kDUp);

            climberConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
            climberConfigs.CurrentLimits.StatorCurrentLimit = 40;

            climberConfigsLead.CurrentLimits.StatorCurrentLimitEnable = true;
            climberConfigsLead.CurrentLimits.StatorCurrentLimit = 40;
    
            
            climberConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            climberConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.Setpoints.topLimit;
            climberConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            climberConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.Setpoints.bottomLimit;

            climberConfigsLead.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            climberConfigsLead.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.Setpoints.topLimit;
            climberConfigsLead.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            climberConfigsLead.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.Setpoints.bottomLimit;
          

            climberLead.getConfigurator().apply(climberConfigsLead);
            climberFollow.getConfigurator().apply(climberConfigs);

            motorStatorCurrent = climberLead.getStatorCurrent();
            motorPosition = climberLead.getPosition();

            BaseStatusSignal.setUpdateFrequencyForAll(50, motorPosition, motorStatorCurrent);
            //climberLead.optimizeBusUtilization();


        }
    
        @Override
        public void updateInputs(ClimberIOInputs inputs) {
            inputs.kPUp = slot0Configs.kP;
            inputs.kIUp = slot0Configs.kI;
            inputs.kDUp = slot0Configs.kD;
            inputs.kFFUp = ClimberConstants.Gains.kFFUp;
    
            /*inputs.kPDown = ClimberConstants.Gains.kPDown;
            inputs.kIDown = ClimberConstants.Gains.kIDown;
            inputs.kDDown = ClimberConstants.Gains.kDDown;
            inputs.kFFDown = ClimberConstants.Gains.kFFDown;*/

            BaseStatusSignal.refreshAll(motorStatorCurrent, motorPosition);
            // Updates all of the inputs/data points being monitored about the motor
            inputs.climberMotorStatorCurrent = motorStatorCurrent.getValueAsDouble();
            inputs.climberMotorPosition = motorPosition.getValueAsDouble();

            inputs.newMotorCurrent = climberIntake.getStatorCurrent().getValueAsDouble();
            inputs.newMotorVoltage = climberIntake.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void configurePID(double kP, double kI, double kD){

      System.out.println("[Climber] Applying PID Values: kP=" + kP + " kI=" + kI + " kD=" + kD);
      // Feedback gains
      slot0Configs.kP = kP;
      slot0Configs.kI = kI;
      slot0Configs.kD = kD;

      // Update the motors with the new Gains
      climberLead.getConfigurator().apply(slot0Configs, 0.050);
      climberFollow.getConfigurator().apply(slot0Configs, 0.050);
    }

    public void stop(){
        climberLead.stopMotor();
        climberFollow.stopMotor();
    }

    @Override
    public void setPO(double PO) {
      DutyCycleOut m_request = new DutyCycleOut(PO);
      climberLead.setControl(m_request);

    }

    @Override
    public void setClimberIntakePO(double PO){
      DutyCycleOut m_request = new DutyCycleOut(PO);
      climberIntake.setControl(m_request);
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

    @Override
    public void runPosition(double positionRad) {

      climberLead.setControl(
        positionVoltage
              .withPosition(positionRad));
    }

    @Override
    public void runPosition(double positionRad, double feedforward) {

      climberLead.setControl(
        positionVoltage
              .withPosition(positionRad)
              .withFeedForward(feedforward));
    }
}