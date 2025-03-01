package frc.lib.util;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;


/** Sets motor usage for a Spark Max motor controller */
public class CANSparkMaxUtil {
  public enum Usage {
    kAll,
    kPositionOnly,
    kVelocityOnly,
    kMinimal
  };

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedback to enable. kAll is the default when a SparkMax is
   *     constructed.
   * @param enableFollowing Whether to enable motor following.
   */
  public static void setCANSparkMaxBusUsage(
    SparkMax motor, Usage usage, boolean enableFollowing) {
    // Yeni bir SparkMaxConfig oluştur
    SparkMaxConfig config = new SparkMaxConfig();

    // Frame periyotlarını ayarla
    if (enableFollowing) {
      config.signals.primaryEncoderPositionPeriodMs(10);
    } else {
      config.signals.primaryEncoderPositionPeriodMs(500);
    }

    if (usage == Usage.kAll) {
      config.signals.primaryEncoderVelocityPeriodMs(20);
    } else if (usage == Usage.kPositionOnly) {
      config.signals.primaryEncoderVelocityPeriodMs(500);
    } else if (usage == Usage.kVelocityOnly) {
      config.signals.primaryEncoderVelocityPeriodMs(20);
    } else if (usage == Usage.kMinimal) {
      config.signals.primaryEncoderVelocityPeriodMs(500);
    }

    // Yapılandırmayı motora uygula
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedback to enable. kAll is the default when a SparkMax is
   *     constructed.
   */
  public static void setCANSparkMaxBusUsage(SparkMax motor, Usage usage) {
    setCANSparkMaxBusUsage(motor, usage, false);
  }
}
