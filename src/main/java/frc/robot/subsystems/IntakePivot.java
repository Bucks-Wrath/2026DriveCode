package frc.robot.subsystems;

import frc.lib.models.*;
import frc.robot.DeviceIds;
import frc.robot.Robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase implements IPositionControlledSubsystem {

	private boolean isHoldingPosition = false;

    // Set Different Heights
	private double homePosition = 0;
	private double maxUpTravelPosition = 0;

	public double upPositionLimit = maxUpTravelPosition;
	public double downPositionLimit = -32;
	private double targetPosition = 0;
    private MotionMagicDutyCycle targetPositionDutyCycle = new MotionMagicDutyCycle(0);
	private double feedForward = 0.0;
	public double shooterAddValue;

	private final static double onTargetThreshold = 0.1;
		
	private TalonFX IntakePivotFalcon = new TalonFX(DeviceIds.IntakePivot.LeadMotorId, "canivore");

    private TalonFXConfiguration IntakePivotFXConfig = new TalonFXConfiguration();

	public IntakePivot() {
		// Clear Sticky Faults
		this.IntakePivotFalcon.clearStickyFaults();
		
        /** IntakePivot Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		IntakePivotFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        IntakePivotFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Current Limiting */
		IntakePivotFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        IntakePivotFXConfig.CurrentLimits.StatorCurrentLimit = 35;

        /* PID Config */
        IntakePivotFXConfig.Slot0.kP = 0.2;
        IntakePivotFXConfig.Slot0.kI = 0;
        IntakePivotFXConfig.Slot0.kD = 0.01;

        /* Open and Closed Loop Ramping */
        IntakePivotFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        IntakePivotFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        IntakePivotFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        IntakePivotFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;

        //Config Acceleration and Velocity
        IntakePivotFXConfig.MotionMagic.withMotionMagicAcceleration(500);
        IntakePivotFXConfig.MotionMagic.withMotionMagicCruiseVelocity(500);

        // Config Motor
        IntakePivotFalcon.getConfigurator().apply(IntakePivotFXConfig);
        IntakePivotFalcon.getConfigurator().setPosition(0.0);
    }

	public void motionMagicControl() {
		this.manageMotion(targetPosition);
        targetPositionDutyCycle.withPosition(targetPosition);
        targetPositionDutyCycle.withFeedForward(feedForward);
		this.IntakePivotFalcon.setControl(targetPositionDutyCycle);
	}

	public double getCurrentPosition() {
		return this.IntakePivotFalcon.getRotorPosition().getValueAsDouble();
	}

	public double getCurrentDraw() {
		return this.IntakePivotFalcon.getSupplyCurrent().getValueAsDouble();
	}

	public boolean isHoldingPosition() {
		return this.isHoldingPosition;
	}

	public void setIsHoldingPosition(boolean isHoldingPosition) {
		this.isHoldingPosition = isHoldingPosition;
	}

	public double getTargetPosition() {
		return this.targetPosition;
	}

	public boolean setTargetPosition(double position) {
		if (!isValidPosition(position)) {
			return false;
		} else {
			this.targetPosition = position;
			return true;
		}
	}

	public void forceSetTargetPosition(double position) {
		this.targetPosition = position;
	}

	public void incrementTargetPosition(double increment) {
		double currentTargetPosition = this.targetPosition;
		double newTargetPosition = currentTargetPosition + increment;
		if (isValidPosition(newTargetPosition)) {		// && isIntakePivotSafe(newTargetPosition) check for other subsystems
			this.targetPosition = newTargetPosition;
		}
	}

	public boolean isValidPosition(double position) {
		boolean withinBounds = position <= upPositionLimit && position >= downPositionLimit;
		return withinBounds;
	}

    // communicate with commands
	public double getHomePosition() {
		return this.homePosition;
	}

	public double getMaxUpTravelPosition() {
		return this.maxUpTravelPosition;
	}

	public double getFeedForward() {
		return this.feedForward;
	}

	public void resetIntakePivotEncoder() {
        try {
			IntakePivotFalcon.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("IntakePivot.resetIntakePivotEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public double JoystickIntakePivot(){
		double value = 0;
		value = -Robot.m_robotContainer.getOperatorRightStickY();
		return value;
	}

	public double getPositionError() {
		double currentPosition = this.getCurrentPosition();
		double targetPosition = this.getTargetPosition();
		double positionError = Math.abs(currentPosition - targetPosition);
		return positionError;
	}

	public void manageMotion(double targetPosition) {
		double currentPosition = getCurrentPosition();
		if (currentPosition < targetPosition) {
				// set based on gravity
		}
		else {
				//set based on gravity
		}
	}

	public void zeroTarget() {
		targetPosition = 0;
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("IntakePivot Position", this.getCurrentPosition());
		SmartDashboard.putNumber("IntakePivot Target Position", this.getTargetPosition());
		SmartDashboard.putNumber("IntakePivot Position Error", this.getPositionError());
		SmartDashboard.putNumber("IntakePivot Velocity", this.getCurrentVelocity());
		SmartDashboard.putNumber("IntakePivot Current", this.getCurrentDraw());
	}

	@Override
	public double getCurrentVelocity() {
		double currentVelocity = this.IntakePivotFalcon.getVelocity().getValueAsDouble();
		return currentVelocity;
	}

	@Override
	public boolean isInPosition(double targetPosition) {
		double currentPosition = this.getCurrentPosition();
		double positionError = Math.abs(currentPosition - targetPosition);
		if (positionError < onTargetThreshold) {
			return true;
		} else {
			return false;
		}
	}
}   
