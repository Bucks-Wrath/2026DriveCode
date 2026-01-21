package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIds;

public class Shooter extends SubsystemBase {

	private TalonFX ShooterKraken = new TalonFX(DeviceIds.Shooter.LeadMotorId);
    private TalonFXConfiguration ShooterFXConfig = new TalonFXConfiguration();
    private TalonFX ShooterKrakenFollower = new TalonFX(DeviceIds.Shooter.FollowerMotorId);

    private double speed = 0;


	public Shooter() {
        /** Shooter Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		ShooterFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        ShooterFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Set Followers
		ShooterKrakenFollower.setControl(new Follower(ShooterKraken.getDeviceID(), MotorAlignmentValue.Opposed));


        /* Current Limiting */
        //ShooterFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        //ShooterFXConfig.CurrentLimits.SupplyCurrentLimit = 20;
        //ShooterFXConfig.CurrentLimits.SupplyCurrentThreshold = 30;
        //ShooterFXConfig.CurrentLimits.SupplyTimeThreshold = 0.01;

        ShooterFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        ShooterFXConfig.CurrentLimits.StatorCurrentLimit = 25;

        /* PID Config */
        ShooterFXConfig.Slot0.kP = 0.2;
        ShooterFXConfig.Slot0.kI = 0;
        ShooterFXConfig.Slot0.kD = 0;

        /* Open and Closed Loop Ramping */
        ShooterFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
        ShooterFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;

        ShooterFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
        ShooterFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

        // Config Motor
        ShooterKraken.getConfigurator().apply(ShooterFXConfig);
        ShooterKraken.getConfigurator().setPosition(0.0);
        ShooterKrakenFollower.getConfigurator().setPosition(0.0);
	}

	public void setSpeed(double speed) {
        this.ShooterKraken.set(speed);
        this.speed = speed;
	}

    public double getSpeed() {
        return this.speed;
	}

    public void changeShooterSpeed(double adjustmentValue){
		this.setSpeed(adjustmentValue + this.getSpeed());
	}

	public double getCurrentDrawLeader() {
		return this.ShooterKraken.getSupplyCurrent().getValueAsDouble();
	}

    public double getCurrentDrawFollower() {
		return this.ShooterKrakenFollower.getSupplyCurrent().getValueAsDouble();
	}

	public void resetShooterEncoder() {
        try {
			ShooterKraken.getConfigurator().setPosition(0.0);
            ShooterKrakenFollower.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("Shooter.resetShooterEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Shooter Current", this.getCurrentDrawLeader());
        SmartDashboard.putNumber("Shooter Follower Current", this.getCurrentDrawFollower());
	}
}