// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.utils;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableNumber implements DoubleSupplier {
  private static final String tableKey = "TunableNumbers";

  private final String key;
  private boolean hasDefault = false;
  private double defaultValue;
  private LoggedDashboardNumber dashboardNumber; //deprecated?
  private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  public TunableNumber(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }


  public TunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  //Set the default value of the number. The default value can only be set once.
  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      if (Constants.tuningMode) {
        dashboardNumber = new LoggedDashboardNumber(key, defaultValue);
      }
    }
  }

  //Get the current value, from dashboard if available and in tuning mode.
  public double get() {
    if (!hasDefault) {
      return 0.0;
    } else {
      return Constants.tuningMode ? dashboardNumber.get() : defaultValue;
    }
  }

  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  public static void ifChanged(
      int id, Consumer<double[]> action, TunableNumber... tunableNumbers) {
    if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
      action.accept(Arrays.stream(tunableNumbers).mapToDouble(TunableNumber::get).toArray());
    }
  }

  //Runs action if any of the tunableNumbers have changed
  public static void ifChanged(int id, Runnable action, TunableNumber... tunableNumbers) {
    ifChanged(id, values -> action.run(), tunableNumbers);
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}