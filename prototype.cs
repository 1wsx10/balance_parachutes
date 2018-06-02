public Program() {}

int runCounter = 0;
public void Main() {
	runCounter += 1;

	Echo("a");
	IMyParachute chute = (IMyParachute)GridTerminalSystem.GetBlockWithName("chute");
	IMyShipController cont = (IMyShipController)GridTerminalSystem.GetBlockWithName("controller");
	IMyGyro gyro = (IMyGyro)GridTerminalSystem.GetBlockWithName("gyro");
	// IMyTextPanel panel = (IMyTextPanel)GridTerminalSystem.GetBlockWithName("panel");
	log("", false);
Echo("a");
	log(cont.CustomName);
	Echo("a");
	log(gyro.CustomName);
	// log(panel.CustomName);
Echo("a");
	log(chute.CustomName);
	log($"Door: {chute.Status}");
	log($"Door ratio: {chute.OpenRatio.Round(2)}");
	log($"Atmo: {chute.Atmosphere.Round(2)}");

Echo("a");

	MyShipMass shipMass = cont.CalculateShipMass();
Echo("a");


	// V = ((2*m*g)/(ρ*A*C))^(1/2)
	// m*g = (A * C * ρ * V^2)/2
	// A*C = (2*m*g) / (ρ * V^2)

	//     m = the mass of the falling object.
	//     g = the acceleration due to gravity. ...
	//     ρ = the density of the fluid the object is falling through.
	//     A = the projected area of the object. ...
	//     C = the drag coefficient.
	//     V = the terminal velocity
Echo("a");
	double grav = cont.GetNaturalGravity().Length();
	double speed = cont.GetShipSpeed();
	float mass = shipMass.PhysicalMass;
	float density = chute.Atmosphere;
Echo("a");
	double area_drag = (2 * mass * grav) / (Math.Pow(speed, 2) * density);
	double force = (area_drag * density * Math.Pow(speed, 2)) / 2;
Echo("a");
	log($"area_drag: {area_drag}");
	log($"force: {force}");
	log($"actual force: {calcChuteForce(chute)}");

	if(runCounter % 60 == 0) {
		Me.CustomData = Me.CustomData + $"{density}, {area_drag}\n";
	}

	// 2592.713
	// 2667.56064837799
	// 2644.61377713933
	// 2704.37801200463

}

/**
 *	gyro = gyroscope to set required torque to
 *	torque = desired torque
 *	vel = Current Rotational Velocity
 */
public const int fticks = 60;
public const float scale = 48.2288857f;
public Vector3D rotate_Torque(IMyGyro gyro, Vector3D torque, Vector3D vel) {

	//TODO get this
	Matrix inverseInertiaTensor = this.m_grid.Physics.RigidBody.InverseInertiaTensor;
	//some 'general overall measure of size and mass distribution'
	Vector3 inertiaDiag = new Vector3(inverseInertiaTensor.M11, inverseInertiaTensor.M22, inverseInertiaTensor.M33);

	Vector3D velDiff_Ramp = torque * inertiaDiag;

	Vector3D velDiff = (fticks + Math.Sqrt(fticks*fticks - 4 * scale * velDiff_Ramp * velDiff_Ramp)) / (2 * scale * velDiff_Ramp);

	Vector3D velOverride = velDiff + vel;
	return velOverride;
}

/**	GAME CODE
 *
 * decompiled and annotated
 * names changed to indicate what they are
 *
 *
private void UpdateOverriddenGyros()
{
	//if we have the ability to use gyros
	if (this.ResourceSink.SuppliedRatio > 0f && this.m_grid.Physics.Enabled && !this.m_grid.Physics.RigidBody.IsFixed)
	{
		//calculate difference between angular velocity and desired angular velocity
		Matrix matrix = this.m_grid.PositionComp.WorldMatrixInvScaled.GetOrientation();
		this.m_grid.WorldMatrix.GetOrientation();
		Vector3 w_angularVelocity = Vector3.Transform(this.m_grid.Physics.AngularVelocity, ref matrix);
		Vector3 w_angularVelocityDiff = this.m_overrideTargetVelocity - w_angularVelocity;

		if (w_angularVelocityDiff == Vector3.Zero)
		{
			//no change in angular velocity vs desired, stop
			return;
		}

		//change in angular velocity vs desired, maintain acceleration ramp
		this.UpdateOverrideAccelerationRampFrames(w_angularVelocityDiff);
		//if w_angularVelocityDiff.Length() > 1.5
		//	m_overrideAccelerationRampFrames = 120
		//else
		//	0 < m_overrideAccelerationRampFrames < 120


		//velocityDiff / (60 / (0 < x <= 120))
		//between 0 and 2 seconds from when we were activated
		Vector3 w_angularVelocityDiffRamp = w_angularVelocityDiff * (60f / (float)this.m_overrideAccelerationRampFrames.Value);
		Matrix inverseInertiaTensor = this.m_grid.Physics.RigidBody.InverseInertiaTensor;
		//some 'general overall measure of size and mass distribution'
		Vector3 inverseInertiaTensorDiagonal = new Vector3(inverseInertiaTensor.M11, inverseInertiaTensor.M22, inverseInertiaTensor.M33);
		//rotational energy = rotationalVelocity^2 * intertiaTensorDiagonal / 2
		Vector3 overrideTorque = w_angularVelocityDiffRamp / inverseInertiaTensorDiagonal;


		//overrideForce + forceAvailableAfterUsedByControl
		float availableTorque = this.m_maxOverrideForce + this.m_maxGyroForce * (1f - this.ControlTorque.Length());
		Vector3 allowedOverrideTorque = Vector3.ClampToSphere(overrideTorque, availableTorque);
		this.Torque = this.ControlTorque * this.m_maxGyroForce + allowedOverrideTorque;

		//im thinking 'this.ControlTorque' must be limited by the gyros that don't have override checked

		this.Torque *= this.ResourceSink.SuppliedRatio;
		if (this.Torque.LengthSquared() < 0.0001f)
		{
			return;
		}
		this.m_grid.Physics.AddForce(MyPhysicsForceType.ADD_BODY_FORCE_AND_BODY_TORQUE, null, null, new Vector3?(this.Torque), null, true, false);
	}
}

private void UpdateOverrideAccelerationRampFrames(Vector3 velocityDiff)
{
	if (this.m_overrideAccelerationRampFrames.HasValue)
	{
		if (this.m_overrideAccelerationRampFrames > 1)
		{
			this.m_overrideAccelerationRampFrames--;
		}
		return;
	}
	float num = velocityDiff.LengthSquared();
	if (num > 2.467401f)
	{
		this.m_overrideAccelerationRampFrames = new int?(120);
		return;
	}
	this.m_overrideAccelerationRampFrames = new int?((int)(num * 48.2288857f) + 1);
}
/**/


public const float radiusMultiplier = 8f;
public const float reefAtmosphereLevel = 0.5f;// opening atmospheric pressure
public const float dragCoefficient = 1f;

public Vector3D calcChuteForce(IMyParachute chute) {
	Vector3D chuteVelocity = chute.GetVelocity();

	if (!(chuteVelocity.LengthSquared() > 4f)) {
		return Vector3D.Zero;
	}

	Vector3D chuteDirection = Vector3D.Normalize(chuteVelocity);
	Vector3D forceDirection = -chuteDirection;

	double num = 10.0 * (double)(chute.Atmosphere - reefAtmosphereLevel)/* * ((double)this.m_parachuteAnimationState / 50.0)*/;
	double radius = num * radiusMultiplier * (double)chute.CubeGrid.GridSize / 2.0;
	if(radius < 0.0f || chuteVelocity.LengthSquared() <= 1f) {
		return Vector3D.Zero;
	}

	double area = 3.1415926535897931 * radius * radius;
	double force = 2.5 * (chute.Atmosphere * 1.225) * (double)chuteVelocity.LengthSquared() * area * dragCoefficient;

	log($"Area: {2.5 * (chute.Atmosphere * 1.225) * (double)chuteVelocity.LengthSquared() * dragCoefficient / force}");

	if(force > 0) {
		return Vector3D.Multiply(forceDirection, force);
	} else {
		return Vector3D.Zero;
	}
}


public void Save() {}


// not efficient, just a simple debugging tool
public const bool debug = true;
public void log(string text, bool append = true) {
	if(!debug) return;
	List<IMyTextPanel> panels = new List<IMyTextPanel>();

	GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(panels, block => block is IMyTextPanel && block.CustomName.ToLower().Contains("debug"));

	if(append) {
		text += "\n";
	}

	foreach(IMyTextPanel panel in panels) {
		panel.WritePublicText(text, append);
	}
}



}
public static class CustomProgramExtensions {

	public static double dot(this Vector3D a, Vector3D b) {
		return Vector3D.Dot(a, b);
	}

	public static Vector3D project(this Vector3D a, Vector3D b) {
		double adb = a.dot(b);
		double bdb = b.dot(b);
		return b * adb / bdb;
	}

	public static Vector3D reject(this Vector3D a, Vector3D b) {
		return Vector3D.Reject(a, b);
	}

	public static void setThrust(this IMyThrust thruster, Vector3D desired) {
		var proj = desired.project(thruster.WorldMatrix.Backward);

		if(proj.dot(thruster.WorldMatrix.Backward) > 0) {//negative * negative is positive... so if its greater than 0, you ignore it.
			thruster.ThrustOverride = 0;
			return;
		}

		thruster.ThrustOverride = (float)proj.Length();
	}

	public static void setThrust(this IMyThrust thruster, Vector3D desired, out string error) {
		error = "";
		var proj = desired.project(thruster.WorldMatrix.Backward);

		if(proj.dot(thruster.WorldMatrix.Backward) > 0) {//negative * negative is positive... so if its greater than 0, you ignore it.
			thruster.ThrustOverride = 0;
			error += "wrong way";
			return;
		}
		error += $"right way";
		error += $"\nproportion: {(proj.Length() / desired.Length()).Round(2)}";
		error += $"\nproj: {proj.Length().Round(1)}";
		error += $"\ndes: {desired.Length().Round(1)}";

		error += $"\nproj: {proj.Round(1)}";
		error += $"\ndesired: {desired.Round(1)}";

		thruster.ThrustOverride = (float)proj.Length();
	}


	public static Vector3D Round(this Vector3D vec, int num) {
		return Vector3D.Round(vec, num);
	}

	public static double Round(this double val, int num) {
		return Math.Round(val, num);
	}

	public static float Round(this float val, int num) {
		return (float)Math.Round(val, num);
	}

	public static Vector3D getWorldMoveIndicator(this IMyShipController controller) {
		return Vector3D.TransformNormal(controller.MoveIndicator, controller.WorldMatrix);
	}

	public static bool IsNaN(this double val) {
		return double.IsNaN(val);
	}

	public static Vector3D NaNtoZero(this Vector3D val) {
		if(val.X.IsNaN()) {
			val.X = 0;
		}
		if(val.Y.IsNaN()) {
			val.Y = 0;
		}
		if(val.Z.IsNaN()) {
			val.Z = 0;
		}

		return val;
	}

	public static Vector3 GetVector(this Base6Directions.Direction dir) {
		return Base6Directions.GetVector(dir);
	}

	public static Vector3D TransformNormal(this Vector3D vec, MatrixD mat) {
		return Vector3D.TransformNormal(vec, mat);
	}
