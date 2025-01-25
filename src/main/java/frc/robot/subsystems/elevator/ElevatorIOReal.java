package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {

    private final TalonFX elevatorLead = new TalonFX(13);
    private final TalonFX elevatorFollow = new TalonFX(14);



    private double positionTargetSetpoint;

    public ElevatorIOReal() {
        setup();
        setFollow();

    }

    public void setup(){

    }

    public void setFollow(){
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
    inputs.kPUp = ElevatorConstants.PID.kPUp;
    inputs.kIUp = ElevatorConstants.PID.kIUp;
    inputs.kDUp = ElevatorConstants.PID.kDUp;
    inputs.kFFUp = ElevatorConstants.PID.kFFUp;
    
    inputs.kPDown = ElevatorConstants.PID.kPDown;
    inputs.kIDown = ElevatorConstants.PID.kIDown;
    inputs.kDDown = ElevatorConstants.PID.kDDown;
    inputs.kFFDown = ElevatorConstants.PID.kFFDown;

    inputs.targetPosition = this.positionTargetSetpoint;
   // inputs.appliedVoltage = elevatorLead.getAppliedOutput();
    inputs.outputCurrentAmps = getCurrent();
    inputs.position = getPosition();
    inputs.velocity = getVelocity();
    }

    @Override
    public void stop() {
    elevatorLead.stopMotor();
    }

    @Override
    public void configurePID(double kP, double kI, double kD, double kFF) {
    //  elevatorPIDController.setP(kP);
    //  elevatorPIDController.setI(kI);
    //  elevatorPIDController.setI(kD);
    //  elevatorPIDController.setFF(kFF);
    }
  
    @Override
    public double getPosition() {
     // return this.elevatorEncoder.getPosition();
     return 0.0;
    }
  
    @Override
    public void setPosition(double targetPosition) {
      this.positionTargetSetpoint = targetPosition;
      manageMotion(targetPosition);
      //elevatorPIDController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
    }
  
     @Override
    public void setPO(double PO) {
      elevatorLead.set(PO);
    }
  
    @Override
    public double getVelocity() {
      //return elevatorLead.getEncoder().getVelocity();
      return 0.0;
    }
  
    @Override
    public double getCurrent() {
      //return elevatorLead.getOutputCurrent();
      return 0.0;

    }
  
    private void manageMotion(double targetPosition) {
      double currentPosition = getPosition();
        if (currentPosition > targetPosition) {
          configurePID(ElevatorConstants.PID.kPUp, ElevatorConstants.PID.kIUp, ElevatorConstants.PID.kDUp, ElevatorConstants.PID.kFFUp);
        }
        else {
          configurePID( ElevatorConstants.PID.kPDown, ElevatorConstants.PID.kIDown, ElevatorConstants.PID.kDDown, ElevatorConstants.PID.kFFDown);
        }
    }
  
   
  
  }
  
