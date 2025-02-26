package frc.robot.subsystems.elevator;

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
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOKraken implements ElevatorIO {
    private TalonFX elevatorLead;
    private TalonFX elevatorFollow;

    TalonFXConfiguration elevatorConfigs = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    private StatusSignal<Current> motorStatorCurrent;
    private StatusSignal<Angle> motorPosition;

    private final PositionVoltage positionVoltage = new PositionVoltage(0.0);


    public ElevatorIOKraken(){

      elevatorLead = new TalonFX(13);
      elevatorFollow = new TalonFX(14);

      elevatorFollow.setControl(new Follower(elevatorLead.getDeviceID(), true));
      //elevatorLead.setControl(new Follower(elevatorFollow.getDeviceID(), true));

      elevatorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      elevatorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; //NeutralModeValue.Brake;
      configurePID(ElevatorConstants.Gains.kPUp,ElevatorConstants.Gains.kIUp,ElevatorConstants.Gains.kDUp);

      elevatorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
      elevatorConfigs.CurrentLimits.StatorCurrentLimit = 40;

      elevatorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      elevatorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.Setpoints.topLimit;
      elevatorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      elevatorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.Setpoints.bottomLimit;

      elevatorLead.getConfigurator().apply(elevatorConfigs);
      elevatorFollow.getConfigurator().apply(elevatorConfigs);

      motorStatorCurrent = elevatorLead.getStatorCurrent();
      motorPosition = elevatorLead.getPosition();
      BaseStatusSignal.setUpdateFrequencyForAll(50, motorPosition, motorStatorCurrent);
      //elevatorLead.optimizeBusUtilization();
    }


    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
      inputs.kPUp = ElevatorConstants.Gains.kPUp;
      inputs.kIUp = ElevatorConstants.Gains.kIUp;
      inputs.kDUp = ElevatorConstants.Gains.kDUp;
      inputs.kFFUp = ElevatorConstants.Gains.kFFUp;

      inputs.kPDown = ElevatorConstants.Gains.kPDown;
      inputs.kIDown = ElevatorConstants.Gains.kIDown;
      inputs.kDDown = ElevatorConstants.Gains.kDDown;
      inputs.kFFDown = ElevatorConstants.Gains.kFFDown;

      BaseStatusSignal.refreshAll(motorStatorCurrent, motorPosition);
      // Updates all of the inputs/data points being monitored about the motor
      inputs.elevatorMotorStatorCurrent = motorStatorCurrent.getValueAsDouble();
      inputs.elevatorMotorPosition = motorPosition.getValueAsDouble(); //elevatorLead.getPosition().getValueAsDouble();

    }

    @Override
    public void configurePID(double kP, double kI, double kD){

      System.out.println("[Elevator] Applying PID Values: kP=" + kP + " kI=" + kI + " kD=" + kD);
      // Feedback gains
      slot0Configs.kP = kP;
      slot0Configs.kI = kI;
      slot0Configs.kD = kD;

      // Update the motors with the new Gains
      elevatorLead.getConfigurator().apply(slot0Configs, 0.050);
      elevatorFollow.getConfigurator().apply(slot0Configs, 0.050);
    }

    @Override
    public void stop(){
        elevatorLead.stopMotor();
    }

    @Override
    public void setPO(double PO) {
      DutyCycleOut m_request = new DutyCycleOut(PO);

      elevatorLead.setControl(m_request);
      //elevatorFollow.setControl(m_request);
    }

    @Override
    public void setPosition(double position) {
      elevatorLead.setPosition(position);
    }

    @Override
    public double getPosition() {
      return motorPosition.getValueAsDouble();
    }

    @Override
    public double getVelocity() {
      return elevatorLead.getVelocity().getValueAsDouble();
    }

    @Override
    public void runPosition(double positionRad) {

      elevatorLead.setControl(
        positionVoltage
              .withPosition((positionRad)));
    }

    @Override
    public void runPosition(double positionRad, double feedforward) {

      elevatorLead.setControl(
        positionVoltage
              .withPosition((positionRad))
              .withFeedForward(feedforward));
    }
}