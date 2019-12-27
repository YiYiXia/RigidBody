#include"RigidBody.h"

RigidBody::RigidBody()
{
	omega = 0;
	angacceleration = 0;
	acceleration = Vector2d::Zero();
	gravity = Vector2d(0, -9);
	collision = false;
}

RigidBody::~RigidBody()
{

}

void RigidBody::InitialBarycenter()
{
	sumMass = 0;
	Vector2d sumPosition = Vector2d::Zero();
	Vector2d sumMassVelocity = Vector2d::Zero();

	for (int i = 0; i < pointList.size(); i++)
	{
		sumMass += pointList[i].mass;
		sumPosition += pointList[i].mass*pointList[i].position;
		sumMassVelocity += pointList[i].mass*pointList[i].velocity;
	}

	if (sumMass < 1e-12)
	{
		barycenter = Vector2d::Zero();
		barycenterVelocity = Vector2d::Zero();
	}
	else
	{
		barycenter = sumPosition / sumMass;
		barycenterVelocity = sumMassVelocity / sumMass;
	}

	InertiaMoment = 0;
	for (int i = 0; i < pointList.size(); i++)
	{
		Vector2d dist = pointList[i].position - barycenter;
		InertiaMoment += pointList[i].mass*dist.norm()*dist.norm();
	}
}

void RigidBody::AddPoint(Vector2d position, double mass, Vector2d velocity)
{
	MassPoint p;
	p.mass = mass;
	p.position = position;
	p.velocity = velocity;
	pointList.push_back(p);
}

void RigidBody::Rotate(double deltaTheta)
{
	Matrix2d rot = Matrix2d(2, 2);
	rot(0, 0) = cos(deltaTheta);
	rot(0, 1) = -sin(deltaTheta);
	rot(1, 0) = sin(deltaTheta); 
	rot(1, 1) = cos(deltaTheta);
	cout << rot << endl;
	for (int i = 0; i < pointList.size(); i++)
	{
		pointList[i].position = rotateCenter + rot*(pointList[i].position - rotateCenter);
	}
	//Update barycenter
	barycenter = rotateCenter + rot*(barycenter - rotateCenter);
}

double RigidBody::GetInertiaMoment(Vector2d p)
{
	Vector2d d = p - barycenter;
	return InertiaMoment + sumMass*d.norm()*d.norm();
}

void RigidBody::Update(double DT)
{
	omega += angacceleration*DT;
	Rotate(omega*DT);


}

void RigidBody::Solve()
{
	double Inertia = GetInertiaMoment(rotateCenter);
	Vector2d d = barycenter - rotateCenter;
	if (collision == false)
		torque = d[0] * sumMass*gravity[1] - d[1] * sumMass*gravity[0];
	else
		torque = 0;
	angacceleration = torque / Inertia;
	barycenterVelocity[0] = -omega*d[1];
	barycenterVelocity[1] = omega*d[0];
	for (int i = 0; i < pointList.size(); i++)
	{
		Vector2d d = pointList[i].position - rotateCenter;
		pointList[i].velocity = Vector2d(-omega*d[1], omega*d[0]);
	}
}

void RigidBody::Collision()
{
	double Inertia = GetInertiaMoment(rotateCenter);
	double Energy = 0.5*Inertia*omega*omega;

	for (int i = 0; i < pointList.size(); i++)
	{
		collision = false;
		//boundary information should be change later.
		if (pointList[i].position[1] < (1 + 1e-12) && (rotateCenter[0] != pointList[i].position[0] || rotateCenter[1] != pointList[i].position[1]) && pointList[i].velocity[1] <= 0)
		{
			collision = true;
			rotateCenter = pointList[i].position;
			//use energy to get the new omega;
			Energy = 0.58*Energy;
			Inertia = GetInertiaMoment(rotateCenter);
			omega = (omega > 0 ? 1 : -1)*sqrt(2 * Energy / Inertia);
			if (abs(omega) < 0.04) omega = 0;//this maybe an artifact.
			cout << omega << endl;
			break;
		}
	}
}

void RigidBody::CollisionRigid(RigidBody obj)
{
	for (int i = 0; i < obj.pointList.size(); i++)
	{
		Vector2d p = obj.pointList[i].position;
		SDFinfo info = Distance(p);
		if (info.distance < 1e-3)
		{
			/*Vector2d mv=*/
		}
	}
}

int RigidBody::Contains(double x, double y)
{
	int result = 1;
	int len = pointList.size();
	for (int i = 0, j = len - 1; i<len; j = i++)
	{
		Vector2d &vi = pointList[i].position, &vj = pointList[j].position;
		if ((vi[1] > y) != (vj[1] > y) && (x < (vj[0] - vi[0]) * (y - vi[1]) / (vj[1] - vi[1]) + vi[0])) //前半部分：点被夹在一条线段的范围内 后半部分：点与线段上的临近点比较
		{
			result *= -1;
		}
	}
	return result;
}

double RigidBody::SegmentSDF(double x, double y, double sx, double sy, double ex, double ey)
{
	double vx = x - sx, vy = y - sy;
	double ux = ex - sx, uy = ey - sy;
	double p = fmax(fmin((ux*vx + uy*vy) / (ux*ux + uy*uy), 1), 0);
	double px = sx + p*ux, py = sy + p*uy;
	return sqrt((x - px)*(x - px) + (y - py)*(y - py));
}

SDFinfo RigidBody::Distance(Vector2d point)
{
	SDFinfo info;
	double d = 10000;
	int n = pointList.size() - 1, m = 0;
	for (int i = pointList.size() - 1, j = 0; j < pointList.size(); i = j, j++)
	{
		double l = SegmentSDF(point[0], point[1], pointList[i].position[0], pointList[i].position[1], pointList[j].position[0], pointList[j].position[1]);
		if (d > l)
		{
			d = l;
			m = i;
			n = j;
		}
	}
	int sign = Contains(point[0], point[1]);
	info.distance = sign*d;
	info.num_s = m;
	info.num_e = n;
	return info;
}


Vector2d RigidBody::Gradient(double x, double y)
{
	double nx, ny;
	nx = (Distance(Vector2d(x + EPSILON, y)).distance - Distance(Vector2d(x - EPSILON, y)).distance)*0.5 / EPSILON;
	ny = (Distance(Vector2d(x, y + EPSILON)).distance - Distance(Vector2d(x, y - EPSILON)).distance)*0.5 / EPSILON;
	double l = sqrt(nx*nx + ny*ny);
	nx /= l;
	ny /= l;
	return Vector2d(nx, ny);
}

