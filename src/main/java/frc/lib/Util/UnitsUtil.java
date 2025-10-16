package frc.lib.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class UnitsUtil {
    public static <M extends Measure<U>, U extends Unit> boolean isNear(M expected, M actual, M tolerance) {
        return MathUtil.isNear(expected.baseUnitMagnitude(), actual.baseUnitMagnitude(), tolerance.baseUnitMagnitude());
    } 

    /**
	 * Class used store translate distances in the form of angles. Used for elevators to interface with the IO layer which only supports angles.
	 */
	public static class DistanceAngleConverter {
		private final Distance radius;

		public DistanceAngleConverter(Distance radius) {
			this.radius = radius;
		}

		/**
		 * Converts a Distance measurement to an equal Angle measurement based on radius initialized with.
		 *
		 * @param distance Distance to convert to Angle.
		 * @return Angle the Distance is equivalent to.
		 */
		public Angle toAngle(Distance distance) {
			return Units.Radians.of(distance.baseUnitMagnitude() / radius.baseUnitMagnitude());
		}

		/**
		 * Converts a Linear Velocity measurement to an equal Angular Velocity measurement based on radius initialized with.
		 *
		 * @param distance Linear Velocity to convert to Angular Velocity.
		 * @return Angular Velocity the Linear Velocity is equivalent to.
		 */
		public AngularVelocity toAngle(LinearVelocity distance) {
			return toAngle(distance.times(BaseUnits.TimeUnit.one())).per(BaseUnits.TimeUnit);
		}

		/**
		 * Converts a Linear Acceleration measurement to an equal Angular Acceleration measurement based on radius initialized with.
		 *
		 * @param distance Linear Acceleration to convert to Angular Acceleration.
		 * @return Angular Acceleration the Linear Acceleration is equivalent to.
		 */
		public AngularAcceleration toAngle(LinearAcceleration distance) {
			return toAngle(distance.times(BaseUnits.TimeUnit.one())).per(BaseUnits.TimeUnit);
		}

		/**
		 * Converts an Angle measurement to an equal Distance measurement based on radius initialized with.
		 *
		 * @param distance Angle to convert to Distance.
		 * @return Distance the Angle is equivalent to.
		 */
		public Distance toDistance(Angle angle) {
			return BaseUnits.DistanceUnit.of(angle.in(Units.Radians) * radius.baseUnitMagnitude());
		}

		/**
		 * Converts an Angular Velocity measurement to an equal Linear Velocity measurement based on radius initialized with.
		 *
		 * @param distance Angular Velocity to convert to Linear Velocity.
		 * @return Linear Velocity the Angular Velocity is equivalent to.
		 */
		public LinearVelocity toDistance(AngularVelocity angle) {
			return toDistance(angle.times(BaseUnits.TimeUnit.one())).per(BaseUnits.TimeUnit);
		}

		/**
		 * Converts an Angular Acceleration measurement to an equal Linear Acceleration measurement based on radius initialized with.
		 *
		 * @param distance Angular Acceleration to convert to Linear Acceleration.
		 * @return Linear Acceleration the Angular Acceleration is equivalent to.
		 */
		public LinearAcceleration toDistance(AngularAcceleration angle) {
			return toDistance(angle.times(BaseUnits.TimeUnit.one())).per(BaseUnits.TimeUnit);
		}

		/**
		 * Gets an angle unit equivalent to a distance unit with the conversion of the radius initialized with.
		 *
		 * @param unit The distance unit to convert.
		 * @return The distance represented as an AngleUnit
		 */
		public AngleUnit getDistanceUnitAsAngleUnit(DistanceUnit unit) {
			return Units.derive(BaseUnits.AngleUnit)
					.aggregate(toAngle(unit.one()).baseUnitMagnitude())
					.named(unit.name())
					.symbol(unit.symbol())
					.make();
		}

		/**
		 * Gets a distance unit equivalent to a angle unit with the conversion of the radius initialized with.
		 *
		 * @param unit The angle unit to convert.
		 * @return The distance represented as a DistanceUnit
		 */
		public DistanceUnit getAngleUnitAsDistanceUnit(AngleUnit unit) {
			return Units.derive(BaseUnits.DistanceUnit)
					.splitInto(toDistance(unit.one()).baseUnitMagnitude())
					.named(unit.name())
					.symbol(unit.symbol())
					.make();
		}

		public Distance getDrumRadius() {
			return radius;
		}
	}
}
