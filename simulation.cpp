/*-----------------------------------------------------------
  Simulation Source File
  -----------------------------------------------------------*/
#include"stdafx.h"
#include"simulation.h"
#include <iostream>
#include <map>
#include <vector>
#include <algorithm>

/*-----------------------------------------------------------
  macros
  -----------------------------------------------------------*/
#define SMALL_VELOCITY		(0.01f)

/*-----------------------------------------------------------
  globals
  -----------------------------------------------------------*/
/*
vec2	gPlaneNormal_Left(1.0,0.0);
vec2	gPlaneNormal_Top(0.0,1.0);
vec2	gPlaneNormal_Right(-1.0,0.0);
vec2	gPlaneNormal_Bottom(0.0,-1.0);
*/

table gTable;

static const float gRackPositionX[] = {0.0f,0.0f,(BALL_RADIUS*2.0f),(-BALL_RADIUS*2.0f),(BALL_RADIUS*4.0f)}; 
static const float gRackPositionZ[] = {0.5f,0.0f,(-BALL_RADIUS*3.0f),(-BALL_RADIUS*3.0f)}; 

float gCoeffRestitution = 0.5f;
float gCoeffFriction = 0.03f;
float gGravityAccn = 9.8f;


/*-----------------------------------------------------------
  cushion class members
  -----------------------------------------------------------*/
void cushion::MakeNormal(void)
{
	//can do this in 2d
	vec2 temp = vertices[1]-vertices[0];
	normal(0) = temp(1);
	normal(1) = -temp(0);
	normal.Normalise();
}

void cushion::MakeCentre(void)
{
	centre = vertices[0];
	centre += vertices[1];
	centre/=2.0;
}

/*-----------------------------------------------------------
  ball class members
  -----------------------------------------------------------*/
int ball::ballIndexCnt = 0;

void ball::ApplyImpulse(vec2 imp)
{
	velocity = imp;
	//hitBall = true;
}

void ball::ApplyFrictionForce(int ms)
{
	if(velocity.Magnitude()<=0.0) return;

	//accelaration is opposite to direction of motion
	vec2 accelaration = -velocity.Normalised();
	//friction force = constant * mg
	//F=Ma, so accelaration = force/mass = constant*g
	accelaration *= (gCoeffFriction * gGravityAccn);
	//integrate velocity : find change in velocity
	vec2 velocityChange = ((accelaration * ms)/1000.0f);
	//cap magnitude of change in velocity to remove integration errors
	if(velocityChange.Magnitude() > velocity.Magnitude()) velocity = 0.0;
	else velocity += velocityChange;
}

void ball::DoBallCollision(ball &b)
{
	if(HasHitBall(b)) HitBall(b);
}

void ball::DoPlaneCollision(const cushion &b)
{
	if(HasHitPlane(b)) HitPlane(b);
	//if (HasHitPlane(b)) AddBall();
}

void ball::Update(int ms)
{
	//apply friction
	ApplyFrictionForce(ms);
	//integrate position
	position += ((velocity * ms)/1000.0f);
	//set small velocities to zero
	if(velocity.Magnitude()<SMALL_VELOCITY) velocity = 0.0;

	//gTable.parts.AddBall(ms);
}

bool ball::HasHitPlane(const cushion &c) const
{
	//if moving away from plane, cannot hit
	if(velocity.Dot(c.normal) >= 0.0) return false;
	
	//if in front of plane, then have not hit
	vec2 relPos = position - c.vertices[0];
	double sep = relPos.Dot(c.normal);
	if(sep > radius) return false;
	return true;
}

bool ball::HasHitBall(const ball &b) const
{
	//work out relative position of ball from other ball,
	//distance between balls
	//and relative velocity
	vec2 relPosn = position - b.position;
	float dist = (float) relPosn.Magnitude();
	vec2 relPosnNorm = relPosn.Normalised();
	vec2 relVelocity = velocity - b.velocity;

	//if moving apart, cannot have hit
	if(relVelocity.Dot(relPosnNorm) >= 0.0) return false;
	//if distnce is more than sum of radii, have not hit
	if(dist > (radius+b.radius)) return false;
	return true;
}

void ball::HitPlane(const cushion &c)
{
	//reverse velocity component perpendicular to plane  
	double comp = velocity.Dot(c.normal) * (1.0+gCoeffRestitution);
	vec2 delta = -(c.normal * comp);
	velocity += delta;
}

void ball::HitBall(ball &b)
{
	//find direction from other ball to this ball
	vec2 relDir = (position - b.position).Normalised();

	//split velocities into 2 parts:  one component perpendicular, and one parallel to 
	//the collision plane, for both balls
	//(NB the collision plane is defined by the point of contact and the contact normal)
	float perpV = (float)velocity.Dot(relDir);
	float perpV2 = (float)b.velocity.Dot(relDir);
	vec2 parallelV = velocity-(relDir*perpV);
	vec2 parallelV2 = b.velocity-(relDir*perpV2);
	
	//Calculate new perpendicluar components:
	//v1 = (2*m2 / m1+m2)*u2 + ((m1 - m2)/(m1+m2))*u1;
	//v2 = (2*m1 / m1+m2)*u1 + ((m2 - m1)/(m1+m2))*u2;
	float sumMass = mass + b.mass;
	float perpVNew = (float)((perpV*(mass-b.mass))/sumMass) + (float)((perpV2*(2.0*b.mass))/sumMass);
	float perpVNew2 = (float)((perpV2*(b.mass-mass))/sumMass) + (float)((perpV*(2.0*mass))/sumMass);
	
	//find new velocities by adding unchanged parallel component to new perpendicluar component
	velocity = parallelV + (relDir*perpVNew);
	b.velocity = parallelV2 + (relDir*perpVNew2);
}

void ballSet::AddBall(void)
{
	if (index >= NUM_BALLS)
	{
		CalcScore();
		index = 0;
		AddBall();
		return;
	}
	balls[index] = new ball;
	balls[index]->position(0) = gTable.camID * SHEET_SEP;
	balls[index]->position(1) = 0.9;
	balls[index]->velocity = 0.0;

	index++;
}

void ballSet::CalcScore(void)
{
	double min = 100;
	int minIndex = 0;
	int ballIndex[NUM_BALLS];
	double dis[NUM_BALLS];
	int sortedIndex[NUM_BALLS];
	double multiplier = gTable.camID * SHEET_SEP;

	//RESET ALL SCORES TO 0
	//for (int i = 0; i < NUM_TEAMS; i++)
	//{
	//	score[i] = 0;
	//}

	for (int i = 0; i < index; i++)
	{
		double posX = balls[i]->position(0);
		double posY = balls[i]->position(1);

		double output = sqrt(pow(posX - (0.0 + multiplier), 2) + pow(posY + 1.6, 2) * 1.0);

		ballIndex[i] = i; 
		dis[i] = output;
	}
	for (int i = 0; i < index; i++)
	{
		for (int j = 0; j < index; j++)
		{
			//find smallest value and save it plus the index
			if (dis[j] < min && dis[j] != 100) { min = dis[j]; minIndex = ballIndex[j]; }
		}
		sortedIndex[i] = minIndex;
		dis[minIndex] = 100;
		min = 100;
	}
	if (sortedIndex[0] % 2 == 0)
	{
		score[0 + 2 * gTable.camID] += 1;
		for (int i = 1; i < index; i++)
		{
			if (sortedIndex[i] % 2 == 0) { score[0 + 2 * gTable.camID] += 1; }
			else { break; }
		}
	}
	else {
		score[1 + 2 * gTable.camID] += 1;
		for (int i = 1; i < index; i++)
		{
			if (sortedIndex[i] % 2 != 0) { score[1 + 2 * gTable.camID] += 1; }
			else { break; }
		}
	}
}
/*-----------------------------------------------------------
  table class members
  -----------------------------------------------------------*/
void table::SetupCushions(void)
{
	int sheetID = 0;
	for (int i = 0; i < NUM_CUSHIONS; i += 4)
	{
		float tempID = sheetID * SHEET_SEP;
		//Left Flat
		cushions[i] = new cushion;
		cushions[i]->vertices[0](0) = -TABLE_X + tempID;
		cushions[i]->vertices[0](1) = -TABLE_Z - 1;
		cushions[i]->vertices[1](0) = -TABLE_X + tempID;
		cushions[i]->vertices[1](1) = TABLE_Z;
		//Bottom Flat
		cushions[i + 1] = new cushion;
		cushions[i + 1]->vertices[0](0) = -TABLE_X + tempID;
		cushions[i + 1]->vertices[0](1) = TABLE_Z;
		cushions[i + 1]->vertices[1](0) = TABLE_X + tempID;
		cushions[i + 1]->vertices[1](1) = TABLE_Z;
		//Right Flat
		cushions[i + 2] = new cushion;
		cushions[i + 2]->vertices[0](0) = TABLE_X + tempID;
		cushions[i + 2]->vertices[0](1) = TABLE_Z;
		cushions[i + 2]->vertices[1](0) = TABLE_X + tempID;
		cushions[i + 2]->vertices[1](1) = -TABLE_Z - 1;
		//Top Flat
		cushions[i + 3] = new cushion;
		cushions[i + 3]->vertices[0](0) = TABLE_X + tempID;
		cushions[i + 3]->vertices[0](1) = -TABLE_Z - 1;
		cushions[i + 3]->vertices[1](0) = -TABLE_X + tempID;
		cushions[i + 3]->vertices[1](1) = -TABLE_Z - 1;

		sheetID++;
	}

	for(int i=0;i<NUM_CUSHIONS;i++)
	{
		cushions[i]->MakeCentre();
		cushions[i]->MakeNormal();
	}
}

void table::Update(int ms)
{
	//check for collisions for each ball
	for (int i = 0; i < parts.index; i++)
	{
		for (int j = tableID; j < tableID + 4; j++)
		{
			parts.balls[i]->DoPlaneCollision(*cushions[j]);
		}

		for (int j = (i + 1); j < parts.index; j++)
		{
			parts.balls[i]->DoBallCollision(*parts.balls[j]);
		}
	}
	//update all balls
	for (int i = 0; i < parts.index; i++) parts.balls[i]->Update(ms);
}

bool table::AnyBallsMoving(void) const
{
	//return true if any ball has a non-zero velocity
	for (int i = 0; i < parts.index; i++)
	{
		if (parts.balls[i]->velocity(0) != 0.0) return true;
		if (parts.balls[i]->velocity(1) != 0.0) return true;
	}
	return false;
}

bool table::ballHit(void) const
{
	if (parts.balls[parts.index - 1]->position(0) != camID * SHEET_SEP) return true;
	if (parts.balls[parts.index - 1]->position(1) != 0.9) return true;
}