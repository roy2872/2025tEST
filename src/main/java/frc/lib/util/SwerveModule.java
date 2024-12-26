package frc.lib.util;

import java.lang.module.Configuration;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.CTREConfigs;
import frc.robot.Constants;
import swervelib.parser.SwerveDriveConfiguration;


public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward swerveFeedforward = new SimpleMotorFeedforward(0, 0,0);

    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(CTREConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        angleMotor = new TalonFX(moduleConstants.angleMotorID);
        angleMotor.getConfigurator().apply(CTREConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        driveMotor.getConfigurator().apply(CTREConfigs.swerveDriveFXConfig);
        driveMotor.getConfigurator().setPosition(0.0);
    }


    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        System.out.println(angleOffset.getRotations() + " and " + getCANcoder().getRotations());
        System.out.println(absolutePosition);
        System.out.println(angleMotor.setPosition(absolutePosition));
        System.out.println(angleMotor.getPosition());
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        angleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

     private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            driveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = swerveFeedforward.calculate(desiredState.speedMetersPerSecond);
            driveMotor.setControl(driveVelocity);
        }
    }

      public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(driveMotor.getVelocity().getValueAsDouble(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(driveMotor.getPosition().getValueAsDouble(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble())
        );
    }

}
