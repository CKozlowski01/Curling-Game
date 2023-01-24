/*-----------------------------------------------------------
  Simulation Header File
 -----------------------------------------------------------*/
#include"vecmath.h"

/*-----------------------------------------------------------
  Macros
  Definitions of table start points, ball radius etc.
 -----------------------------------------------------------*/
#define TABLE_X			(0.6f) 
#define TABLE_Z			(1.2f)
#define TABLE_Y			(0.1f)
#define BALL_RADIUS		(0.05f)
#define BALL_MASS		(0.1f)
#define TWO_PI			(6.2832f)
#define	SIM_UPDATE_MS	(10)
#define NUM_SHEETS		(5)
#define NUM_TEAMS		(2 * NUM_SHEETS)
#define NUM_BALLS		(8)
#define NUM_CUSHIONS	(4 * NUM_SHEETS)
#define NUM_POCKETS     (4)
#define POCKET_RADIUS   (0.1)
#define SHEET_SEP		(1.3)


/*-----------------------------------------------------------
  cushion class
 -----------------------------------------------------------*/
class cushion
{
public:
	//Includes the verteces for one point e.g x and y axis
	vec2	vertices[2]; //2d
	//Includes the centre point
	vec2	centre;
	//Not a clue
	vec2	normal;

	//Calls the makeNormal and makeCentre functions from simulation.cpp
	void MakeNormal(void);
	void MakeCentre(void);
};

/*-----------------------------------------------------------
  ball class
 -----------------------------------------------------------*/
class ball
{
	static int ballIndexCnt;
public:
	vec2	position;
	vec2	velocity;
	float	radius;
	float	mass;
	int		index;
	//Contructor to initialise the ball object
	ball() : position(0.0), velocity(0.0), radius(BALL_RADIUS),
		mass(BALL_MASS) {
		index = ballIndexCnt++;
	}

	void ApplyImpulse(vec2 imp);
	void ApplyFrictionForce(int ms);
	void DoPlaneCollision(const cushion& c);
	void DoBallCollision(ball& b);
	void Update(int ms);

	bool HasHitPlane(const cushion& c) const;
	bool HasHitBall(const ball& b) const;

	void HitPlane(const cushion& c);
	void HitBall(ball& b);
};

class ballSet
{
public:
	ball*	balls[NUM_BALLS];
	int		score[NUM_TEAMS];
	int		index;
	int*	scoreP;

	//Contructor to initialise the ball set
	ballSet()
	{
		for (int i = 0; i < NUM_BALLS; i++) balls[i] = 0;
		index = 0;
		AddBall();
	}
	//Destructor to delete the ball set
	~ballSet()
	{
		for (int i = 0; i < NUM_BALLS; i++)
		{
			if (balls[i]) delete balls[i];
		}
	}
	void AddBall();
	void CalcScore();
};


/*-----------------------------------------------------------
  table class
  -----------------------------------------------------------*/
class table
{
public:
	int tableID;
	int camID;
	//Instantiates the correct number of balls and cushions
	cushion* cushions[NUM_CUSHIONS];	
	ballSet parts;

	void SetupCushions(void);
	void Update(int ms);	
	bool AnyBallsMoving(void) const;
	bool ballHit(void) const;
};

/*-----------------------------------------------------------
  global table
  -----------------------------------------------------------*/
extern table gTable;
