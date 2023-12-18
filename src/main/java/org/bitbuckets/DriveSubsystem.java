package org.bitbuckets;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.commands.CircleCommand;

public class DriveSubsystem implements Subsystem {


    final WPI_TalonFX talonFX1;
    final WPI_TalonFX talonFX2;


    final SimpleMotorFeedforward ff;
    final DifferentialDriveKinematics kinematics;
    final XboxController joystick;

    static final double ks = 6;
    static final double kv = 7.8053;

    public DriveSubsystem(WPI_TalonFX talonFX1, WPI_TalonFX talonFX2, SimpleMotorFeedforward ff, DifferentialDriveKinematics kinematics, XboxController joystick) {
        this.talonFX1 = talonFX1;
        this.talonFX2 = talonFX2;
        this.ff = ff;
        this.kinematics = kinematics;
        this.joystick = joystick;

        register();
    }

    @Override
    public void periodic() {

    }

    public void ourTeleopPeriod() {

        //SETTING WHEELS SPEEDS TO JOYSTICK AXES
        double forwardSpeed = joystick.getLeftY();
        double rotationSpeed = joystick.getRightX();

        if (Math.abs(forwardSpeed) < 0.05) {
            forwardSpeed = 0;
        }

        if (Math.abs(rotationSpeed) < 0.05) {
            rotationSpeed = 0;
        }
        System.out.println("F: " + forwardSpeed + "    |    R: " + rotationSpeed);


        driveAt(forwardSpeed, rotationSpeed);
    }


    public void driveAt(double forward, double rotate) {
        //GETTING WHEEL SPEEDS FROM CHASSIS SPEEDS
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, 0.0, rotate, new Rotation2d());
        double leftSpeeds = kinematics.toWheelSpeeds(speeds).leftMetersPerSecond;
        double rightSpeeds = kinematics.toWheelSpeeds(speeds).rightMetersPerSecond;
        //CALC FF FROM WHEELS SPEEDS
        double leftFF = ff.calculate(leftSpeeds);
        double rightFF = ff.calculate(rightSpeeds);
        //SET THE CALC VOLTAGE FOR EACH MOTOR CONTROLLER
        talonFX1.setVoltage(leftFF);
        talonFX2.setVoltage(rightFF);
    }
}
