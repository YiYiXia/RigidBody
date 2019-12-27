/*
* RigidBody
* This class is used to simulate the polygon object with a simple model that 
* the mass of the polygon is in the point.
*
* Yiming Xia 2019.12.26
*/

#pragma once
#include<Eigen/Dense>
#include<iostream>
#include<vector>
#define EPSILON 0.01f

using namespace Eigen;
using namespace std;


/*
mass point is an easy model to represent the polygon boundary.
*/
struct MassPoint 
{
	double mass;
	Vector2d position;
	Vector2d velocity;
	MassPoint()
	{
		mass = 0;
		position = Vector2d::Zero();
		velocity = Vector2d::Zero();
	}

};

//The information of the point with the polygon
typedef struct SDFinfo
{
	double distance;
	int num_s;                           //与之最近的线段编号
	int num_e;
} SDFinfo;

class RigidBody 
{
public:
	RigidBody();
	~RigidBody();
	
	void InitialBarycenter();												//Initial position , velocity and inertia.
	void AddPoint(Vector2d position, double mass, Vector2d velocity);       //Add point into list.
	void Rotate(double deltaTheta);											//Rotate the rigid body.
	double GetInertiaMoment(Vector2d p);									//Calculate the inertia with Parallel axis theorem
	void Update(double DT);												    //Update position.
	void Solve();
	void Collision();
	int Contains(double x, double y);										//Use to judge whether a polygon contain the point.
	void CollisionRigid(RigidBody obj);										


	vector<MassPoint> pointList;											//use to save polygon info.
	Vector2d barycenter;													//barycenter coordinate.
	Vector2d barycenterVelocity;											//barycenter veloctiy.
	
	double sumMass;															//sum mass.
	double InertiaMoment;													//inertia.
	double omega;															//omega value.

	double angacceleration;                           //
	Vector2d acceleration;

	Vector2d rotateCenter;

	Vector2d Force;
	double torque;
private:
	bool collision;
	Vector2d gravity;
	double SegmentSDF(double x, double y, double sx, double sy, double ex, double ey);
	SDFinfo Distance(Vector2d point);
	Vector2d Gradient(double x, double y);
};