// Pool Game.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include<glut.h>
#include<math.h>
#include<cstring>
#include"simulation.h"

//cue variables
float gCueAngle = 0.0;
float gCuePower = 0.25;
bool gCueControl[4] = {false,false,false,false};
float gCueAngleSpeed = 2.0f; //radians per second
float gCuePowerSpeed = 0.25f;
float gCuePowerMax = 0.75;
float gCuePowerMin = 0.1;
float gCueBallFactor = 8.0;
bool gDoCue = true;
int j = 0;

//camera variables
vec3 gCamPos(0.0,2.0,2.1);
vec3 gCamLookAt(0.0,0.0,0.0);
bool gCamRotate = true;
float gCamRotSpeed = 0.2;
float gCamMoveSpeed = 0.5;
bool gCamL = false;
bool gCamR = false;
bool gCamU = false;
bool gCamD = false;
bool gCamZin = false;
bool gCamZout = false;

//slate variable
int slateNum = 0;

//rendering options
#define DRAW_SOLID	(1)

void DoCamera(int ms)
{
	static const vec3 up(0.0,1.0,0.0);

	if(gCamRotate)
	{
		if(gCamL)
		{
			vec3 camDir = (gCamLookAt - gCamPos).Normalised();
			vec3 localL = up.Cross(camDir);
			vec3 inc = (localL*((gCamRotSpeed*ms)/100.0) );
			gCamLookAt = gCamPos + camDir + inc;
		}
		if(gCamR)
		{
			vec3 camDir = (gCamLookAt - gCamPos).Normalised();
			vec3 localR = up.Cross(camDir);
			vec3 inc = (localR* ((gCamRotSpeed*ms)/100.0) );
			gCamLookAt = gCamPos + camDir - inc;
		}
		if(gCamU)
		{
			vec3 camDir = (gCamLookAt - gCamPos).Normalised();
			vec3 localR = camDir.Cross(up);
			vec3 localUp = localR.Cross(camDir);
			vec3 inc = (localUp* ((gCamMoveSpeed*ms)/100.0) );
			gCamLookAt = gCamPos + camDir + inc;
		}
		if(gCamD)
		{
			vec3 camDir = (gCamLookAt - gCamPos).Normalised();
			vec3 localR = camDir.Cross(up);
			vec3 localUp = localR.Cross(camDir);
			vec3 inc = (localUp* ((gCamMoveSpeed*ms)/100.0) );
			gCamLookAt = gCamPos + camDir - inc;
		}		
	}
	else
	{
		if(gCamL)
		{
			vec3 camDir = (gCamLookAt - gCamPos).Normalised();
			vec3 localL = up.Cross(camDir);
			vec3 inc = (localL* ((gCamMoveSpeed*ms)/1000.0) );
			gCamPos += inc;
			gCamLookAt += inc;
		}
		if(gCamR)
		{
			vec3 camDir = (gCamLookAt - gCamPos).Normalised();
			vec3 localR = camDir.Cross(up);
			vec3 inc = (localR* ((gCamMoveSpeed*ms)/1000.0) );
			gCamPos += inc;
			gCamLookAt += inc;
		}
		if(gCamU)
		{
			vec3 camDir = (gCamLookAt - gCamPos).Normalised();
			vec3 localR = camDir.Cross(up);
			vec3 localUp = localR.Cross(camDir);
			vec3 inc = (localUp* ((gCamMoveSpeed*ms)/1000.0) );
			gCamPos += inc;
			gCamLookAt += inc;
		}
		if(gCamD)
		{
			vec3 camDir = (gCamLookAt - gCamPos).Normalised();
			vec3 localR = camDir.Cross(up);
			vec3 localDown = camDir.Cross(localR);
			vec3 inc = (localDown* ((gCamMoveSpeed*ms)/1000.0) );
			gCamPos += inc;
			gCamLookAt += inc;
		}
	}

	if(gCamZin)
	{
		vec3 camDir = (gCamLookAt - gCamPos).Normalised();
		vec3 inc = (camDir* ((gCamMoveSpeed*ms)/1000.0) );
		gCamPos += inc;
		gCamLookAt += inc;
	}
	if(gCamZout)
	{
		vec3 camDir = (gCamLookAt - gCamPos).Normalised();
		vec3 inc = (camDir* ((gCamMoveSpeed*ms)/1000.0) );
		gCamPos -= inc;
		gCamLookAt -= inc;
	}
}

void RenderScene(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	//set camera
	glLoadIdentity();
	//gluLookAt(gCamPos(0),gCamPos(1),gCamPos(2),gCamLookAt(0),gCamLookAt(1),gCamLookAt(2),0.0f,1.0f,0.0f);
	gluLookAt(gTable.camID * SHEET_SEP, 2.0f, 2.1f, gTable.camID * SHEET_SEP, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);

	//draw the ball
	glColor3f(1.0,0.0,0.0);
	for(int i=0;i<gTable.parts.index;i++)
	{
		if (i % 2 == 0)
		{
			glPushMatrix();
			glTranslatef(gTable.parts.balls[i]->position(0), (BALL_RADIUS / 2.0), gTable.parts.balls[i]->position(1));
			#if DRAW_SOLID
			glutSolidSphere(gTable.parts.balls[i]->radius, 32, 32);
			//glTranslatef(gTable.balls[i].position(0), (BALL_RADIUS / 2.0), gTable.balls[i].position(1));
			//#if DRAW_SOLID
			//glutSolidSphere(gTable.balls[i].radius, 32, 32);
			#else
			glutWireSphere(gTable.balls[i].radius, 12, 12);
			#endif
			glPopMatrix();
			glColor3f(0.0, 1.0, 0.0);
		}
		else
		{
			glPushMatrix();
			glTranslatef(gTable.parts.balls[i]->position(0), (BALL_RADIUS / 2.0), gTable.parts.balls[i]->position(1));
			#if DRAW_SOLID
			glutSolidSphere(gTable.parts.balls[i]->radius, 32, 32);
			//glTranslatef(gTable.balls[i].position(0), (BALL_RADIUS / 2.0), gTable.balls[i].position(1));
			//#if DRAW_SOLID
			//glutSolidSphere(gTable.balls[i].radius, 32, 32);
			#else
			glutWireSphere(gTable.balls[i].radius, 12, 12);
			#endif
			glPopMatrix();
			glColor3f(1.0, 0.0, 0.0);
		}
	}

	glColor3f(1.0,1.0,1.0);

	//draw the table
	for(int i=0;i<NUM_CUSHIONS;i++)
	{	
		//Draws the 4 lines from (X, Y, Z) going bottom-left, top-left, top-right, bottom-right.
		glBegin(GL_QUADS);
		glVertex3f (gTable.cushions[i]->vertices[0](0), 0.0, gTable.cushions[i]->vertices[0](1));
		glVertex3f (gTable.cushions[i]->vertices[0](0), 0.1, gTable.cushions[i]->vertices[0](1));
		glVertex3f (gTable.cushions[i]->vertices[1](0), 0.1, gTable.cushions[i]->vertices[1](1));
		glVertex3f (gTable.cushions[i]->vertices[1](0), 0.0, gTable.cushions[i]->vertices[1](1));
		glEnd();
	}
	//Draw team scores
	char str[80];
	for (int i = 0; i < NUM_TEAMS; i += 2)
	{
		float x = 0;
		float div = i;
		sprintf(str, "TEAM %d: %d", i + 1, gTable.parts.score[i]);
		x =  (i * SHEET_SEP) / 2;
		glColor3f(1.0, 1.0, 1.0);
		glRasterPos3f(-0.7 + x, 0.0, -2.8);
		int len, j;
		len = (int)strlen(str);
		for (j = 0; j < len; j++) {
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, str[j]);
		}
	}
	for (int i = 1; i < NUM_TEAMS; i += 2)
	{
		float x = 0;
		float div = i;
		sprintf(str, "TEAM %d: %d", i + 1, gTable.parts.score[i]);
		x = ((i - 1) * SHEET_SEP) / 2;
		glColor3f(1.0, 1.0, 1.0);
		glRasterPos3f(-0.7 + x, 0.0, -2.5);
		int len, j;
		len = (int)strlen(str);
		for (j = 0; j < len; j++) {
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, str[j]);
		}
	}

	//Draw Circles on ground
	for (int n = 0; n < NUM_SHEETS; n++)
	{
		for (int i = 0; i < NUM_POCKETS; i++)
		{
			float xn = n * SHEET_SEP;
			glBegin(GL_LINE_LOOP);
			for (int j = 0; j < 100; j++) {
				float theta = TWO_PI * float(j) / float(100);//get the current angle 
				float x = (POCKET_RADIUS * (i + 0.9)) * cosf(theta);//calculate the x component 
				float y = (POCKET_RADIUS * (i + 0.9)) * sinf(theta);//calculate the y component 
				glVertex3f(x + xn, 0, y + -1.6);//output vertex 
			}
			glEnd();
		}

	}

	//draw the cue
	if(gDoCue)
	{
		glBegin(GL_LINES);
		float cuex = sin(gCueAngle) * gCuePower;
		float cuez = cos(gCueAngle) * gCuePower;
		glColor3f(1.0,0.0,0.0);
		glVertex3f (gTable.parts.balls[gTable.parts.index - 1]->position(0), (BALL_RADIUS/2.0f), gTable.parts.balls[gTable.parts.index - 1]->position(1));
		glVertex3f ((gTable.parts.balls[gTable.parts.index - 1]->position(0)+cuex), (BALL_RADIUS/2.0f), (gTable.parts.balls[gTable.parts.index - 1]->position(1)+cuez));
		glColor3f(1.0,0.0,0.0);
		glEnd();
	}

	//glPopMatrix();

	glFlush();
	glutSwapBuffers();
}

void SpecKeyboardFunc(int key, int x, int y) 
{
	switch(key)
	{
		case GLUT_KEY_LEFT:
		{
			gCueControl[0] = true;
			break;
		}
		case GLUT_KEY_RIGHT:
		{
			gCueControl[1] = true;
			break;
		}
		case GLUT_KEY_UP:
		{
			gCueControl[2] = true;
			break;
		}
		case GLUT_KEY_DOWN:
		{
			gCueControl[3] = true;
			break;
		}
	}
}

void SpecKeyboardUpFunc(int key, int x, int y) 
{
	switch(key)
	{
		case GLUT_KEY_LEFT:
		{
			gCueControl[0] = false;
			break;
		}
		case GLUT_KEY_RIGHT:
		{
			gCueControl[1] = false;
			break;
		}
		case GLUT_KEY_UP:
		{
			gCueControl[2] = false;
			break;
		}
		case GLUT_KEY_DOWN:
		{
			gCueControl[3] = false;
			break;
		}
	}
}

void KeyboardFunc(unsigned char key, int x, int y) 
{
	switch(key)
	{
	case(13):
		{
			if(gDoCue)
			{
				vec2 imp(	(-sin(gCueAngle) * gCuePower * gCueBallFactor),
							(-cos(gCueAngle) * gCuePower * gCueBallFactor));
				gTable.parts.balls[gTable.parts.index - 1]->ApplyImpulse(imp);				
			}
			break;
		}
	case(27):
		{
			for(int i=0;i<NUM_BALLS;i++)
			{

			}
			break;
		}
	case(32):
		{
			gCamRotate = false;
			break;
		}
	case('z'):
		{
			gCamL = false;
			break;
		}
	case('c'):
		{
			gCamR = false;
			break;
		}
	case('s'):
		{
			gCamU = false;
			break;
		}
	case('x'):
		{
			gCamD = false;
			break;
		}
	case('f'):
		{
			gCamZin = false;
			break;
		}
	case('v'):
		{
			gCamZout = false;
			break;
		}
	case('1'):
		{
			gTable.tableID = 0;
			gTable.camID = 0;
			gTable.parts.index = 0;
			gTable.parts.AddBall();
			break;
		}
	case('2'):
		{
			gTable.tableID = 4;
			gTable.camID = 1;
			gTable.parts.index = 0;
			gTable.parts.AddBall();
			break;
		}
	case('3'):
		{
			gTable.tableID = 8;
			gTable.camID = 2;
			gTable.parts.index = 0;
			gTable.parts.AddBall();
			break;
		}
	case('4'):
		{
			gTable.tableID = 12;
			gTable.camID = 3;
			gTable.parts.index = 0;
			gTable.parts.AddBall();
			break;
		}
	case('5'):
		{
			gTable.tableID = 16;
			gTable.camID = 4;
			gTable.parts.index = 0;
			gTable.parts.AddBall();
			break;
		}
	}

}

void KeyboardUpFunc(unsigned char key, int x, int y) 
{
	switch(key)
	{
	case(32):
		{
			gCamRotate = true;
			break;
		}
	case('z'):
		{
			gCamL = false;
			break;
		}
	case('c'):
		{
			gCamR = false;
			break;
		}
	case('s'):
		{
			gCamU = false;
			break;
		}
	case('x'):
		{
			gCamD = false;
			break;
		}
	case('f'):
		{
			gCamZin = false;
			break;
		}
	case('v'):
		{
			gCamZout = false;
			break;
		}
	}
}

void ChangeSize(int w, int h) {

	// Prevent a divide by zero, when window is too short
	// (you cant make a window of zero width).
	if(h == 0) h = 1;
	float ratio = 1.0* w / h;

	// Reset the coordinate system before modifying
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	// Set the viewport to be the entire window
	glViewport(0, 0, w, h);

	// Set the correct perspective.
	gluPerspective(45,ratio,0.2,1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//gluLookAt(0.0,0.7,2.1, 0.0,0.0,0.0, 0.0f,1.0f,0.0f);
	gluLookAt(gCamPos(0),gCamPos(1),gCamPos(2),gCamLookAt(0),gCamLookAt(1),gCamLookAt(2),0.0f,1.0f,0.0f);
}

void InitLights(void)
{
	GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat mat_shininess[] = { 50.0 };
	GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
	glClearColor (0.0, 0.0, 0.0, 0.0);
	glShadeModel (GL_SMOOTH);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	GLfloat light_ambient[] = { 2.0, 2.0, 2.0, 1.0 };
	glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_DEPTH_TEST);
}

void UpdateScene(int ms) 
{

	if(gTable.AnyBallsMoving()==false) gDoCue = true;

	else gDoCue = false;

	if(gDoCue)
	{
		if(gCueControl[0]) gCueAngle -= ((gCueAngleSpeed * ms)/1000);
		if(gCueControl[1]) gCueAngle += ((gCueAngleSpeed * ms)/1000);
		if (gCueAngle <0.0) gCueAngle += TWO_PI;
		if (gCueAngle >TWO_PI) gCueAngle -= TWO_PI;

		if(gCueControl[2]) gCuePower += ((gCuePowerSpeed * ms)/1000);
		if(gCueControl[3]) gCuePower -= ((gCuePowerSpeed * ms)/1000);
		if(gCuePower > gCuePowerMax) gCuePower = gCuePowerMax;
		if(gCuePower < gCuePowerMin) gCuePower = gCuePowerMin;
	}

	if (gTable.ballHit() == true && gTable.AnyBallsMoving() == false) gTable.parts.AddBall();

	DoCamera(ms);

	gTable.Update(ms);

	glutTimerFunc(SIM_UPDATE_MS, UpdateScene, SIM_UPDATE_MS);
	glutPostRedisplay();
}

int _tmain(int argc, _TCHAR* argv[])
{
	gTable.SetupCushions();

	glutInit(&argc, ((char **)argv));
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE| GLUT_RGBA);
	glutInitWindowPosition(0,0);
	glutInitWindowSize(1000,700);
	//glutFullScreen();
	glutCreateWindow("MSc Assignment: Curling Game");
	#if DRAW_SOLID
	#endif
	glutDisplayFunc(RenderScene);
	glutTimerFunc(SIM_UPDATE_MS, UpdateScene, SIM_UPDATE_MS);
	glutReshapeFunc(ChangeSize);
	glutIdleFunc(RenderScene);
	
	glutIgnoreKeyRepeat(1);
	glutKeyboardFunc(KeyboardFunc);
	glutKeyboardUpFunc(KeyboardUpFunc);
	glutSpecialFunc(SpecKeyboardFunc);
	glutSpecialUpFunc(SpecKeyboardUpFunc);
	glEnable(GL_DEPTH_TEST);
	glutMainLoop();
}