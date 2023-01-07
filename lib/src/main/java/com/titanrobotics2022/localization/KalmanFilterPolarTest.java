// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.titanrobotics2022.localization.KalmanFilter;

import org.ejml.data.DMatrix2;
import org.ejml.data.DMatrix2x2;
import org.ejml.dense.fixed.NormOps_DDF2;

/**
 * A localizer based around a Kalman filter.
 * 
 * This subsystem should not be declared as a requirement for commands.
 */
public class LocalizationSubsystem extends SubsystemBase {
  private KalmanFilter filter;
  private double step;
  private DMatrix2 mean = new DMatrix2();
  private DMatrix2x2 prec = new DMatrix2x2();
  private WPI_Pigeon2 imu = new WPI_Pigeon2(40);
  private Rotation2d phiOffset = new Rotation2d(Math.PI / 4);
  private Translation2d pigeonBias = new Translation2d(-0.35, -0.59);
  private Rotation2d pigeonOrientation = new Rotation2d();
  private final NetworkTableEntry tv, tx, ty;

  /**
   * Creates a new LocalizationSubsystem.
   * 
   * @param step  The time difference between calls to the periodic method of
   *  this subsystem.
   * @param depth  The maximum derivative of position to consider. For instance,
   *  1 for velocity, or 2 for acceleration.
   * @param drift  The intrinsic covariance due to noise of the highest order
   *  derivative of position considered.
   */
  public LocalizationSubsystem(double step, int depth, double drift) {
    filter = new KalmanFilter(depth, new DMatrix2x2(drift, 0, 0, drift));
    this.step = step;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
  }
  /**
   * Creates a new LocalizationSubsystem.
   * 
   * No intrinsic drift is assumed to exist in the system.
   * 
   * @param step  The time difference between calls to the periodic method of
   *  this subsystem.
   * @param depth  The maximum derivative of position to consider. For instance,
   *  1 for velocity, or 2 for acceleration.
   */
  public LocalizationSubsystem(double step, int depth) {
    this(step, depth, 0.0);
  }
  /**
   * Creates a new LocalizationSubsystem.
   * 
   * Velocity is the highest order derivative of position considered.
   * 
   * @param step  The time difference between calls to the periodic method of
   *  this subsystem.
   * @param drift  The intrinsic covariance due to noise of the highest order
   *  derivative of position considered.
   */
  public LocalizationSubsystem(double step, double drift) {
    this(step, 2, drift);
  }
  /**
   * Creates a new LocalizationSubsystem.
   * 
   * Velocity is the highest order derivative of position considered and no
   * intrinsic drift is assumed to exist in the system.
   * 
   * @param step  The time difference between calls to the periodic method of
   *  this subsystem.
   */
  public LocalizationSubsystem(double step) {
    this(step, 2, 0.0);
  }

  /**
   * Updates the state of the localizer with a new measurement.
   * 
   * @param degree  The order of the derivative of position to update.
   * @param x  The x component of the measurement.
   * @param y  The y component of the measurement.
   * @param varX  The variance in the x component of the measurement.
   * @param varY  The variance in the y component of the measurement.
   * @param covar  The covariance of the x and y components of the measurement.
   */
  public void addData(int degree, double x, double y, double varX, double varY, double covar) {
    mean.setTo(x, y);
    double idet = 1 / (varX * varY - covar * covar);
    prec.setTo(varY * idet, -covar * idet, -covar * idet, varX * idet);
    filter.update(degree, mean, prec);
  }
  /**
   * Updates the state of the localizer with a new measurement.
   * 
   * @param degree  The order of the derivative of position to update.
   * @param pred  The measurement.
   * @param varX  The variance in the x component of the measurement.
   * @param varY  The variance in the y component of the measurement.
   * @param covar  The covariance of the x and y components of the measurement.
   */
  public void addData(int degree, Translation2d pred, double varX, double varY, double covar) {
    addData(degree, pred.getX(), pred.getY(), varX, varY, covar);
  }
  /**
   * Updates the state of the localizer with a new measurement.
   * 
   * @param degree  The order of the derivative of position to update.
   * @param x  The x component of the measurement.
   * @param y  The y component of the measurement.
   * @param var  The variance of the noise in the measurement, which is assumed
   *  to be isotropic.
   */
  public void addData(int degree, double x, double y, double var) {
    mean.setTo(x, y);
    prec.setTo(1/var, 0, 0, 1/var);
    filter.update(degree, mean, prec);
  }
  /**
   * Updates the state of the localizer with a new measurement.
   * 
   * @param degree  The order of the derivative of position to update.
   * @param pred  The measurement.
   * @param var  The variance of the noise in the measurement, which is assumed
   *  to be isotropic.
   */
  public void addData(int degree, Translation2d pred, double var) {
    addData(degree, pred.getX(), pred.getY(), var);
  }

  public void addPosition(Translation2d pos, double varX, double varY, double covar) {
    addData(0, pos, varX, varY, covar);
  }
  public void addPosition(Translation2d pos, double var) {
    addData(0, pos, var);
  }
  public void addVelocity(Translation2d pos, double varX, double varY, double covar) {
    addData(1, pos, varX, varY, covar);
  }
  public void addVelocity(Translation2d pos, double var) {
    addData(1, pos, var);
  }

  /**
   * Sets the current orientation to a specified value.
   * 
   * @param phi  The new current orientation, measured counterclockwise from the
   *  positive x axis.
   */
  public void setOrientation(Rotation2d phi) {
    phiOffset = phiOffset.plus(getOrientation().minus(phi));
  }
  /**
   * Sets the current heading to a specified value.
   * 
   * @param heading  The new current heading, measured clockwise from the
   *  positive y axis.
   */
  public void setHeading(Rotation2d heading) {
    setOrientation(new Rotation2d(Math.PI/2).minus(heading));
  }
  /** Resets the current orientation to zero. */
  public void resetOrientation() {
    setOrientation(new Rotation2d(0));
  }
  /** Resets the current heading to zero. */
  public void resetHeading() {
    setHeading(new Rotation2d(0));
  }

  /**
   * Resets the current position estimate to a specific value and uncertainty.
   * 
   * Warning: This method will overwrite the accumulated position estimate and
   * uncertainty. Use only when necessary.
   * 
   * @param pos  The new current position.
   * @param var  The new position variance.
   */
  public void setPosition(Translation2d pos, double var) {
    mean.setTo(pos.getX(), pos.getY());
    prec.setTo(var, 0, 0, var);
    filter.setPred(0, mean);
    filter.setCov(0, prec);
  }
  /**
   * Resets the current position estimate to a specific value.
   * 
   * Warning: This method will overwrite the accumulated position estimate and
   * uncertainty. Use only when necessary.
   * 
   * The position value is assumed to be precise, without uncertainty.
   * 
   * @param pos  The new current position.
   */
  public void setPosition(Translation2d pos) {
    setPosition(pos, 0);
  }
  /**
   * Resets the current position estimate, preserving uncertainty.
   * 
   * Warning: This method will overwrite the accumulated position estimate. Use
   * only when necessary.
   * 
   * @param pos  The new current position.
   */
  public void translateTo(Translation2d pos) {
    mean.setTo(pos.getX(), pos.getY());
    filter.setPred(0, mean);
  }

  /**
   * Resets the current velocity estimate to a specific value and uncertainty.
   * 
   * Warning: This method will overwrite the accumulated velocity estimate and
   * uncertainty. Use only when necessary.
   * 
   * @param vel  The new current velocity.
   * @param var  The new velocity variance.
   */
  public void setVelocity(Translation2d vel, double var) {
    mean.setTo(vel.getX(), vel.getY());
    prec.setTo(var, 0, 0, var);
    filter.setPred(1, mean);
    filter.setCov(1, prec);
  }
  /**
   * Resets the current velocity estimate to a specific value.
   * 
   * Warning: This method will overwrite the accumulated velocity estimate and
   * uncertainty. Use only when necessary.
   * 
   * The velocity value is assumed to be precise, without uncertainty.
   * 
   * @param vel  The new current velocity.
   */
  public void setVelocity(Translation2d vel) {
    setVelocity(vel, 0);
  }
  /**
   * Resets the current velocity estimate, preserving uncertainty.
   * 
   * Warning: This method will overwrite the accumulated velocity estimate. Use
   * only when necessary.
   * 
   * @param vel  The new current velocity.
   */
  public void accelerateTo(Translation2d vel) {
    mean.setTo(vel.getX(), vel.getY());
    filter.setPred(1, mean);
  }

  /**
   * Returns the current estimate of a given derivative of position.
   * 
   * @param degree  The order of the derivative of position to return the
   *  estimate of.
   * @return  The current estimate of the requested derivative of position.
   */
  public Translation2d getPred(int degree) {
    filter.getPred(degree, mean);
    return new Translation2d(mean.a1, mean.a2);
  }
  /**
   * Returns the current estimate of the position of the robot.
   * 
   * @return  The current estimate of the position of the robot in meters.
   */
  public Translation2d getPosition() {
    return getPred(0);
  }
  /**
   * Returns the current estimate of the velocity of the robot.
   * 
   * @return  The current estimate of the velocity of the robot in meters per
   *  second.
   */
  public Translation2d getVelocity() {
    return getPred(1);
  }

  /**
   * Returns the current estimate of the distance from the origin.
   * 
   * @return  The current estimate of the distance from the origin in meters.
   */
  public double getDistance() {
    filter.getPred(0, mean);
    filter.getCov(0, prec);
    double sqNorm = mean.a1*mean.a1 + mean.a2*mean.a2;
    double lower = Math.sqrt(sqNorm);
    double upper = Math.sqrt(sqNorm + prec.a11 + prec.a22);
    return (upper + lower) / 2;
  }
  /**
   * Returns the current estimate of the theta component of the polar position.
   * 
   * @return  the current estimate of the theta component of the polar position
   *  in radians.
   */
  public Rotation2d getTheta() {
    filter.getPred(0, mean);
    return new Rotation2d(mean.a1, mean.a2);  // TODO: Replace with a more efficient estimator.
  }

  /**
   * Returns the current estimate of the orientation of the robot.
   * 
   * @return  The current estimate of the orientation of the robot measured
   *  counterclockwise from the positive x axis.
   */
  public Rotation2d getOrientation() {
    return imu.getRotation2d().minus(phiOffset);
  }
  /**
   * Returns the current estimate of the heading of the robot.
   * 
   * @return  The current estimate of the heading of the robot, measured
   *  clockwise from the positive y axis.
   */
  public Rotation2d getHeading() {
    return new Rotation2d(Math.PI/2).minus(getOrientation());
  }
  public double getRate() {
    return -imu.getRate() * Math.PI / 180.0;
  }

  /**
   * Estimates the angle from the robot heading to the origin.
   * 
   * @return  The current estimate of the angle from the robot heading to the
   *  origin.
   */
  public Rotation2d getDeltaPhi() {
    double diff = new Rotation2d(Math.PI).minus(getOrientation()).plus(getTheta()).getRadians();
    if(tv.getDouble(0) == 1){
      double cam = Math.toDegrees(tx.getDouble(180));
      double delta = Math.asin(Math.sin(cam - diff));
      diff += delta * 0.25;
    }
    return new Rotation2d(diff);
  }

  private Translation2d accum = new Translation2d();
  private double accum_t = 0;

  private void pigeonUpdate() {
    short[] accArr = new short[]{0, 0, 0};
    imu.getBiasedAccelerometer(accArr);
    Translation2d rawAcc = new Translation2d(9.8 * accArr[0] / (1<<14), 9.8 * accArr[1] / (1<<14));
    Translation2d finalAcc = rawAcc.minus(pigeonBias).rotateBy(pigeonOrientation).rotateBy(getHeading());
    SmartDashboard.putNumber("acc x", rawAcc.getX());
    SmartDashboard.putNumber("acc y", rawAcc.getY());
    accum_t += 0.02;
    accum = accum.plus(rawAcc.minus(accum).times(0.02 / accum_t));
    SmartDashboard.putNumber("accum x", accum.getX());
    SmartDashboard.putNumber("accum y", accum.getY());
    SmartDashboard.putNumber("accum time", accum_t);
    if(accum_t > 60){
      accum_t = 0;
      System.out.println("x: " + accum.getX());
      System.out.println("y: " + accum.getY());
    }
    addData(2, finalAcc, 0.001);
  }

  @Override
  public void periodic() {
    //pigeonUpdate();
    filter.step(step);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}