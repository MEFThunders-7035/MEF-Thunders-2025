package wrapper_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import edu.wpi.first.hal.HAL;
import frc.utils.sim_utils.CANSparkMAXWrapped;
import frc.utils.sim_utils.SparkMAXSimAddon;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SparkMAXWrapperTest {
  @BeforeEach
  public void setUp() {
    HAL.initialize(500, 0); // init HAL just in case
  }

  @Test
  void testSparkMAXWrapper() {
    for (int i = 0; i < 50; i++) {
      CANSparkMAXWrapped sparkMAX =
          new CANSparkMAXWrapped(i, CANSparkMAXWrapped.MotorType.kBrushless);
      assertEquals(sparkMAX, SparkMAXSimAddon.getSparkMAX(i));
      sparkMAX.close();
    }
  }

  @Test
  void testSparkMAXisClosed() {
    CANSparkMAXWrapped sparkMAX =
        new CANSparkMAXWrapped(1, CANSparkMAXWrapped.MotorType.kBrushless);
    sparkMAX.close();
    assertEquals(true, sparkMAX.isThisClosed());
  }

  @SuppressWarnings("resource")
  @Test
  void testSparkMAXClear() {
    new CANSparkMAXWrapped(1, CANSparkMAXWrapped.MotorType.kBrushless);
    SparkMAXSimAddon.resetData(); // reset data already closes the spark max
    assertThrows(IllegalArgumentException.class, () -> SparkMAXSimAddon.getSparkMAX(1));
  }
}
