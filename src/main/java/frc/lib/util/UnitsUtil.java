package frc.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
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
	/*
	 * Checks whether a given measure is within tolerance of the actual value
	 */
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
		public AngleUnit asAngleUnit(DistanceUnit unit) {
			return Units.derive(BaseUnits.AngleUnit)
					.aggregate(toAngle(unit.one()).baseUnitMagnitude())
					.named(unit.name())
					.symbol(unit.symbol())
					.make();
		}

		/**
		 * Gets an angular velocity unit equivalent to a linear velocity unit with the conversion of the radius initialized with.
		 *
		 * @param unit The linear velocity unit to convert.
		 * @return The linear velocity unit represented as an AngularVelocityUnit
		 */
		public AngularVelocityUnit asAngularVelocityUnit(LinearVelocityUnit unit) {
			return Units.derive(BaseUnits.AngleUnit.per(BaseUnits.TimeUnit))
					.aggregate(toAngle(unit.one()).baseUnitMagnitude())
					.named(unit.name())
					.symbol(unit.symbol())
					.make();
		}

		/**
		 * Gets an angular acceleration unit equivalent to a linear acceleration unit with the conversion of the radius initialized with.
		 *
		 * @param unit The linear acceleration unit to convert.
		 * @return The linear acceleration unit represented as an AngularAccelerationUnit
		 */
		public AngularAccelerationUnit asAngularAccelerationUnit(LinearAccelerationUnit unit) {
			return Units.derive(BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).per(BaseUnits.TimeUnit))
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
		public DistanceUnit asDistanceUnit(AngleUnit unit) {
			return Units.derive(BaseUnits.DistanceUnit)
					.splitInto(toDistance(unit.one()).baseUnitMagnitude())
					.named(unit.name())
					.symbol(unit.symbol())
					.make();
		}

		/**
		 * Gets a linear velocity unit equivalent to a angular velocity unit with the conversion of the radius initialized with.
		 *
		 * @param unit The angular velocity unit to convert.
		 * @return The angular velocity unit represented as a LinearVelocityUnit
		 */
		public LinearVelocityUnit asLinearVelocityUnit(AngularVelocityUnit unit) {
			return Units.derive(BaseUnits.DistanceUnit.per(BaseUnits.TimeUnit))
					.splitInto(toDistance(unit.one()).baseUnitMagnitude())
					.named(unit.name())
					.symbol(unit.symbol())
					.make();
		}

		/**
		 * Gets a linear acceleration unit equivalent to a angular acceleration unit with the conversion of the radius initialized with.
		 *
		 * @param unit The angular acceleration unit to convert.
		 * @return The angular acceleration unit represented as a LinearAccelerationUnit
		 */
		public LinearAccelerationUnit asLinearAccelerationUnit(AngularAccelerationUnit unit) {
			return Units.derive(BaseUnits.DistanceUnit.per(BaseUnits.TimeUnit).per(BaseUnits.TimeUnit))
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
