package frc.lib.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class UnitsUtil {
    public static <M extends Measure<U>, U extends Unit> boolean isNear(M expected, M actual, M tolerance) {
        return MathUtil.isNear(expected.baseUnitMagnitude(), actual.baseUnitMagnitude(), tolerance.baseUnitMagnitude());
    } 
}
