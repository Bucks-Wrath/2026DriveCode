package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIds;

public class Agitator extends SubsystemBase {

	private TalonFX AgitatorKraken = new TalonFX(DeviceIds.Agitator.MotorId);
    private TalonFXConfiguration AgitatorFXConfig = new TalonFXConfiguration();


	public Agitator() {
        /** Shooter Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		AgitatorFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        AgitatorFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

     
        /* Current Limiting */
        AgitatorFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        AgitatorFXConfig.CurrentLimits.SupplyCurrentLimit = 20;
        AgitatorFXConfig.CurrentLimits.SupplyCurrentThreshold = 30;
        AgitatorFXConfig.CurrentLimits.SupplyTimeThreshold = 0.01;

        AgitatorFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        AgitatorFXConfig.CurrentLimits.StatorCurrentLimit = 25;

        /* PID Config */
        AgitatorFXConfig.Slot0.kP = 0.2;
        AgitatorFXConfig.Slot0.kI = 0;
        AgitatorFXConfig.Slot0.kD = 0;

        /* Open and Closed Loop Ramping */
        AgitatorFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
        AgitatorFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;

        AgitatorFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
        AgitatorFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

        // Config Motor
        AgitatorKraken.getConfigurator().apply(AgitatorFXConfig);
        AgitatorKraken.getConfigurator().setPosition(0.0);
	}

	public void setSpeed(double speed) {
        this.AgitatorKraken.set(speed);
	}

	public double getCurrentDrawLeader() {
		return this.AgitatorKraken.getSupplyCurrent().getValueAsDouble();
	}

	public void resetShooterEncoder() {
        try {
			AgitatorKraken.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("Shooter.resetShooterEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Agitator Current", this.getCurrentDrawLeader());

	}
}