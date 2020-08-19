#include "glSetup.h"
#include "hsv2rgb.h"

#include <Windows.h>
#include "include/GLFW/glfw3.h"
#include "include/GL/gl.h"
#include "include/GL/glut.h"
#include "include/GL/glew.h"

#pragma comment(lib, "lib/glfw3.lib")
#pragma comment(lib, "lib/opengl32.lib")
#pragma comment(lib, "lib/glut32.lib")
#pragma comment(lib, "lib/glew32.lib")

#include "Eigen/Dense"
using namespace Eigen;

#include <iostream>
using namespace std;

void initialize();
void finalize();

void render(GLFWwindow* window);
void reshape(GLFWwindow* window, int w, int h);
void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods);
void mouse(GLFWwindow* window, int button, int action, int mods);
void cursor(GLFWwindow* window, double xpos, double ypos);

// Colors
GLfloat bgColor[4] = { 1,1,1,1 };

Vector3f ee_goal;
bool keya = false;
bool keyr = false;
bool keyd = false;
bool keyi = false;

// (x, y, z) of control points
int datapoints = 0;
float p[10][3];

int nControlPoints = 0;
Vector3f* controlPoints = NULL;

int main(int argc, char* argv[])
{
	// Keyboard
	cout << "Add 10 control points (a key)" << endl;
	cout << "Select/remove 3 control points (r key)" << endl;
	cout << "Select/drag 2 control points (d key)" << endl;
	cout << "Select edges of the control polygon and insert 2 control points (i key)" << endl << endl;

	// Orthographics viewing
	perspectiveView = false;

	// Initialize the OpenGL system
	GLFWwindow* window = initializeOpenGL(argc, argv, bgColor);
	if (window == NULL) return -1;

	// Callbacks
	glfwSetKeyCallback(window, keyboard);
	glfwSetMouseButtonCallback(window, mouse);
	glfwSetCursorPosCallback(window, cursor);

	// Depth test
	glDisable(GL_DEPTH_TEST);

	// Viewport and perspective setting
	reshape(window, windowW, windowH);

	// Initialization - Main loop
	controlPoints = new Vector3f[14];

	// Main loop
	while (!glfwWindowShouldClose(window))
	{
		render(window); // Draw one frame
		glfwSwapBuffers(window); // Swap buffers
		glfwPollEvents(); // Events
	}

	// Finalization
	finalize();

	// Terminate the glfw system
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}

void initialize()
{
	nControlPoints = datapoints + 4;

	for (int i = 0; i < datapoints; i++)
		controlPoints[i + 2] = Vector3f(p[i][0], p[i][1], p[i][2]);

	for (int i = 0; i < 2; i++)
	{
		controlPoints[i] = controlPoints[2]; // From beginning
		controlPoints[datapoints + 2 + i] = controlPoints[datapoints + 1]; // Before ending
	}
}

void finalize()
{
	delete[] controlPoints;
}

Vector3f BsplinePoint(Vector3f b[4], float t1)
{
	float t2 = t1*t1;
	float t3 = t2*t1;

	float B0 = 1 - 3 * t1 + 3 * t2 - t3;
	float B1 = 4 - 6 * t2 + 3 * t3;
	float B2 = 1 + 3 * t1 + 3 * t2 - 3 * t3;
	float B3 = t3;

	return (b[0] * B0 + b[1] * B1 + b[2] * B2 + b[3] * B3) / 6;
}

void drawBSpline()
{
	int N_POINTS_PER_SEGMENTS = 40;

	// Curve
	glLineWidth(1.5*dpiScaling);

	// Colors
	float hsv[3] = { 0,1,1 };
	float rgb[3];

	Vector3f b[4];
	for (int i = 0; i < nControlPoints - 3; i++)
	{
		hsv[0] = 360.0*i / (nControlPoints - 3); // Degree
		HSV2RGB(hsv, rgb);
		glColor3f(rgb[0], rgb[1], rgb[2]);

		for (int j = 0; j < 4; j++)
			b[j] = controlPoints[i + j];

		// N_POINTS_PER_SEGMENTS points for each curve segment
		glBegin(GL_LINE_STRIP);
		for (int j = 0; j < N_POINTS_PER_SEGMENTS; j++)
		{
			float t = (float)j / (N_POINTS_PER_SEGMENTS - 1);

			Vector3f pt = BsplinePoint(b, t);
			glVertex3fv(pt.data());
		}
		glEnd();
	}

	// Data points
	glPointSize(10);
	glColor3f(1, 0, 0);
	glBegin(GL_POINTS);
	for (int i = 0; i < datapoints; i++)
	{
		glVertex3f(p[i][0], p[i][1], p[i][2]);
	}
	glEnd();
}

void removePoint(float x, float y, float z)
{
	float length = 0.25, dis;
	int index;
	for (int i = 0; i < datapoints; i++)
	{
		dis = sqrt(pow((p[i][0] - x), 2) + pow((p[i][1] - y), 2) + pow((p[i][2] - z), 2));
		if (length > dis) {
			length = dis;
			index = i;
		}
	}

	// Remove data point
	if (length < 0.25)
	{
		for (int j = index; j < datapoints - 1; j++)
		{
			p[j][0] = p[j + 1][0];
			p[j][1] = p[j + 1][1];
			p[j][2] = p[j + 1][2];
		}
		p[datapoints - 1][0] = 0;
		p[datapoints - 1][1] = 0;
		p[datapoints - 1][2] = 0;
		datapoints--;
	}
	if (datapoints == 0)
		keyr = false;
}

int findIndex(float x, float y, float z)
{
	float length = 0.25, dis;
	int index;
	for (int i = 0; i < datapoints; i++)
	{
		dis = sqrt(pow((p[i][0] - x), 2) + pow((p[i][1] - y), 2) + pow((p[i][2] - z), 2));
		if (length > dis) {
			length = dis;
			index = i;
		}
	}

	if (length < 0.25)
		return index;
	else
		return 15;
}

void movePoint(float x, float y, float z)
{
	int index = findIndex(x, y, z);

	if (index != 15)
	{
		p[index][0] = x;
		p[index][1] = y;
		p[index][2] = z;
	}
}

void drawControlPolygon()
{
	int N_POINTS_PER_SEGMENTS = 40;

	// Curve
	glLineWidth(1.0*dpiScaling);
	glColor3f(0, 0, 0);

	for (int i = 0; i < datapoints - 1; i++)
	{
		glBegin(GL_LINE_STRIP);
		for (int j = 0; j < N_POINTS_PER_SEGMENTS; j++)
		{
			//float t = (float)j / (N_POINTS_PER_SEGMENTS - 1);

			float x = (j*p[i + 1][0] + (N_POINTS_PER_SEGMENTS - j)*p[i][0]) / N_POINTS_PER_SEGMENTS;
			float y = (j*p[i + 1][1] + (N_POINTS_PER_SEGMENTS - j)*p[i][1]) / N_POINTS_PER_SEGMENTS;
			float z = (j*p[i + 1][2] + (N_POINTS_PER_SEGMENTS - j)*p[i][2]) / N_POINTS_PER_SEGMENTS;

			glVertex3f(x, y, z);
		}
		glEnd();
	}
}

void selectEdge(float x, float y, float z)
{
	float length = 0.25, dis;
	for (int i = 0; i < datapoints; i++)
	{
		dis = sqrt(pow((p[i][0] - x), 2) + pow((p[i][1] - y), 2) + pow((p[i][2] - z), 2));
		if (length > dis)
			length = dis;
	}

	float length2 = 0.25, dis2;
	float x3, y3, z3;
	int index2;
	int N_POINTS_PER_SEGMENTS = 40;

	for (int i = 0; i < datapoints - 1; i++)
	{
		for (int j = 0; j < N_POINTS_PER_SEGMENTS; j++)
		{
			float x2 = (j*p[i + 1][0] + (N_POINTS_PER_SEGMENTS - j)*p[i][0]) / N_POINTS_PER_SEGMENTS;
			float y2 = (j*p[i + 1][1] + (N_POINTS_PER_SEGMENTS - j)*p[i][1]) / N_POINTS_PER_SEGMENTS;
			float z2 = (j*p[i + 1][2] + (N_POINTS_PER_SEGMENTS - j)*p[i][2]) / N_POINTS_PER_SEGMENTS;

			dis2 = sqrt(pow((x - x2), 2) + pow((y - y2), 2) + pow((z - z2), 2));
			if (dis2 < length2)
			{
				length2 = dis2;
				x3 = x2; y3 = y2; z3 = z2;
				index2 = i;
			}	
		}
	}

	// Add data point
	if (length2 < length && length2 < 0.25)
	{
		for (int i = datapoints; i > index2+1; i--)
		{
			p[i][0] = p[i - 1][0];
			p[i][1] = p[i - 1][1];
			p[i][2] = p[i - 1][2];
		}
		p[index2 + 1][0] = x3;
		p[index2 + 1][1] = y3;
		p[index2 + 1][2] = z3;
		datapoints++;
	}
}

void render(GLFWwindow* window)
{
	// Background color
	glClearColor(bgColor[0], bgColor[1], bgColor[2], bgColor[3]);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Data point
	if (datapoints > 0) {
		initialize();
		drawBSpline();
	}

	if (keyi)
		drawControlPolygon();
}

void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (action == GLFW_PRESS || action == GLFW_REPEAT)
	{
		switch (key)
		{
			// Quit
		case GLFW_KEY_Q:
		case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(window, GL_TRUE); break;

		case GLFW_KEY_A: keya = true; keyr = false; keyd = false; keyi = false; break;
		case GLFW_KEY_R: keya = false; keyr = true; keyd = false; keyi = false; break;
		case GLFW_KEY_D: keya = false; keyr = false; keyd = true; keyi = false; break;
		case GLFW_KEY_I: keya = false; keyr = false; keyd = false; keyi = true; break;
		}
	}
}

bool drag = false;

void mouse(GLFWwindow* window, int button, int action, int mods)
{
	if (action == GLFW_PRESS && button == GLFW_MOUSE_BUTTON_LEFT)
	{
		// In the screen coordinate
		double xpos, ypos;
		glfwGetCursorPos(window, &xpos, &ypos);
		ee_goal = Vector3f(xpos, ypos, 0);

		// In the workspace. See reshape() in glSetup.cpp
		float aspect = (float)screenW / screenH;
		ee_goal[0] = 2.0 * (ee_goal[0] / screenW - 0.5)*aspect;
		ee_goal[1] = -2.0 * (ee_goal[1] / screenH - 0.5);

		if (datapoints == 10)
		{
			keya = false;
		}
		else if (keya) {
			p[datapoints][0] = ee_goal[0];
			p[datapoints][1] = ee_goal[1];
			p[datapoints][2] = 0;
			datapoints++;
		}
		else if (keyi)
		{
			if (datapoints > 1 && datapoints < 11)
				selectEdge(ee_goal[0], ee_goal[1], 0);
		}

		if (datapoints == 0)
			keyr = false;
		else if (keyr)
			removePoint(ee_goal[0], ee_goal[1], 0);

		if (keyd)
			drag = true;
	}
	else if (action == GLFW_RELEASE && button == GLFW_MOUSE_BUTTON_LEFT)
	{
		if (keyd)
			if (drag)
				drag = false;
	}
}

void cursor(GLFWwindow* window, double xpos, double ypos)
{
	if (drag)
	{
		ee_goal = Vector3f(xpos, ypos, 0);

		float aspect = (float)screenW / screenH;
		ee_goal[0] = 2.0 * (ee_goal[0] / screenW - 0.5)*aspect;
		ee_goal[1] = -2.0 * (ee_goal[1] / screenH - 0.5);

		movePoint(ee_goal[0], ee_goal[1], 0);
	}
}