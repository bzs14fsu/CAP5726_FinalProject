//Assignment 3 - Final Project
//Brendan Schneider - bzs14

#include "Helpers.h"

#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <cfloat>
#include <limits>
#include <vector>

#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace std;
using namespace Eigen;


//VBO and VAO Objects
GLuint VBO;
GLuint VAO;


//3d stuff----------------------------

GLFWwindow* window3d;
GLuint ViewTrans3dx, ViewTrans3dy, ViewTrans3dz, Scale3d, Rotation3dx, Rotation3dy, Rotation3dz;
GLuint Transl3d, SelectColor, Barycenter3d, VertexColor, LightSource, PhongShade, WindowSize;

int CurrentObject = -1;
float MousePosX, MousePosY, MousePosZ;

int BunnyVertices, BunnyFaces;
Matrix<float, Dynamic, Dynamic> BunnyArray; //Position(3), Normal(3), Index(1)
MatrixXf BunnyData(11, 1); //Centroid(3), Shading type(1), Scaling(1), Rotation(3), Translate(3)

int OrigVertices, OrigFaces;
Matrix<float, Dynamic, Dynamic> OrigArray;
Matrix<float, Dynamic, Dynamic> OrigVertexArray;


Vector3f ScreenRotation;

void Program3dRun();


void FlatShading(Vector3f v1, Vector3f v2, Vector3f v3)
{
	Vector3f lightsource = RowVector3f(-2, 2, -2), barycenter;
	float x, y, z;
	MatrixXf rotate(3, 3);
	
	//Transforms
	x = BunnyData(5, 0);
	y = BunnyData(6, 0);
	z = BunnyData(7, 0);
	
	barycenter = Vector3f(BunnyData(0, 0), BunnyData(1, 0), BunnyData(2, 0));
	
	rotate.col(0) << cos(z) * cos(y), sin(z) * cos(y), -sin(y);
	rotate.col(1) << cos(z) * sin(y) * sin(x) - sin(z) * cos(x), sin(z) * sin(y) * sin(x) + cos(z) * cos(x), cos(y) * sin(x);
	rotate.col(2) << cos(z) * sin(y) * cos(x) + sin(z) * sin(x), sin(z) * sin(y) * cos(x) - cos(z) * sin(x), cos(y) * cos(x);
	
	v1 = (rotate * ((v1 - barycenter) * BunnyData(4, 0)) + barycenter) +
			Vector3f(BunnyData(8, 0), BunnyData(9, 0), BunnyData(10, 0));
	v2 = (rotate * ((v2 - barycenter) * BunnyData(4, 0)) + barycenter) +
			Vector3f(BunnyData(8, 0), BunnyData(9, 0), BunnyData(10, 0));
	v3 = (rotate * ((v3 - barycenter) * BunnyData(4, 0)) + barycenter) +
			Vector3f(BunnyData(8, 0), BunnyData(9, 0), BunnyData(10, 0));
	
	//Edges
	Vector3f e1, e2, normal, centroid, lightray;
	e1 << v2 - v1;
	e2 << v2 - v3;
	
	//Normal
	normal = e1.cross(e2).normalized();
	
	//Centroid of particular triangle
	centroid << (v1[0] + v2[0] + v3[0]) / 3, (v1[1] + v2[1] + v3[1]) / 3, (v1[2] + v2[2] + v3[2]) / 3;
	lightray = centroid - lightsource;
	float color = max(0.0f, lightray.dot(normal)) * 0.6f;
	
	glUniform3f(VertexColor, color, color, color);
	return;
}

Vector3f CalcNormal(Vector3f v1, Vector3f v2, Vector3f v3)
{
	Vector3f e1, e2, normal;
	e1 << v2 - v1;
	e2 << v2 - v3;
	
	normal = e1.cross(e2).normalized();
	
	return normal;
}

void RemoveFace(int index)
{
	for(int i = index; i < BunnyFaces - 1; i++)
	{
		BunnyArray.col(i * 3) = BunnyArray.col((i + 1) * 3);
		BunnyArray.col(i * 3 + 1) = BunnyArray.col((i + 1) * 3 + 1);
		BunnyArray.col(i * 3 + 2) = BunnyArray.col((i + 1) * 3 + 2);
	}
	
	return;
}

void Contraction()
{
	int numfaces, numpairs = 0;
	float x, y, z, dist;
	
	//Size of contraction
	cout << "How many faces should we contract to? ";
	cin >> numfaces;
	while(numfaces > OrigFaces || numfaces < 0)
	{
		cout << "Invalid Value. Try again. ";
		cin >> numfaces;
	}
	
	
	//Find Q matrix for all vertices
	MatrixXf QArray(4, 4 * OrigVertices);
	QArray = MatrixXf::Zero(4, 4 * OrigVertices);
	MatrixXf Norms(4, OrigFaces);
	Vector3f v1, v2, v3, normal, total;
	float d;
	
	//Equation for each face
	for(int i = 0; i < OrigFaces; i++)
	{
		v1 << OrigArray(0, i * 3), OrigArray(1, i * 3), OrigArray(2, i * 3);
		v2 << OrigArray(0, i * 3 + 1), OrigArray(1, i * 3 + 1), OrigArray(2, i * 3 + 1);
		v3 << OrigArray(0, i * 3 + 2), OrigArray(1, i * 3 + 2), OrigArray(2, i * 3 + 2);
		normal = CalcNormal(v1, v2, v3);
		
		x = normal[0];
		y = normal[1];
		z = normal[2];
		
		d = -x * OrigArray(0, i * 3) - y * OrigArray(1, i * 3) - z * OrigArray(2, i * 3);
		Norms(0, i) = x;
		Norms(1, i) = y;
		Norms(2, i) = z;
		Norms(3, i) = d;
	}
	//Populate QArray
	for(int i = 0; i < OrigVertices; i++)
	{
		for(int j = 0; j < OrigFaces * 3; j++)
		{
			if(OrigArray(6, j) == i)
			{
				QArray.col(i * 4) = QArray.col(i * 4) + Vector4f(Norms(0, j / 3) * Norms(0, j / 3),
					Norms(0, j / 3) * Norms(1, j / 3), Norms(0, j / 3) * Norms(2, j / 3), Norms(0, j / 3) * Norms(3, j / 3));
				
				QArray.col(i * 4 + 1) = QArray.col(i * 4 + 1) + Vector4f(Norms(1, j / 3) * Norms(0, j / 3),
					Norms(1, j / 3) * Norms(1, j / 3), Norms(1, j / 3) * Norms(2, j / 3), Norms(1, j / 3) * Norms(3, j / 3));
				
				QArray.col(i * 4 + 2) = QArray.col(i * 4 + 2) + Vector4f(Norms(2, j / 3) * Norms(0, j / 3),
					Norms(2, j / 3) * Norms(1, j / 3), Norms(2, j / 3) * Norms(2, j / 3), Norms(2, j / 3) * Norms(3, j / 3));
					
				QArray.col(i * 4 + 3) = QArray.col(i * 4 + 3) + Vector4f(Norms(3, j / 3) * Norms(0, j / 3),
					Norms(3, j / 3) * Norms(1, j / 3), Norms(3, j / 3) * Norms(2, j / 3), Norms(3, j / 3) * Norms(3, j / 3));
			}
		}
	}
	
	
	//Find valid pairs (connected or near)
	MatrixXi Pairs(3, OrigVertices * OrigVertices);
	
	for(int i = 0; i < OrigVertices; i++)
	{
		//If connected
		for(int j = 0; j < 3 * OrigFaces; j++)
		{
			if(OrigArray(6, j) == i)
			{
				Pairs(0, numpairs) = i;
				Pairs(1, numpairs) = int(OrigArray(6, (j / 3) * 3));
				Pairs(2, numpairs) = 0;
				if((j / 3) * 3 != j)
					numpairs++;
				
				Pairs(0, numpairs) = i;
				Pairs(1, numpairs) = int(OrigArray(6, (j / 3) * 3 + 1));
				Pairs(2, numpairs) = 0;
				if((j / 3) * 3 + 1 != j)
					numpairs++;
				
				Pairs(0, numpairs) = i;
				Pairs(1, numpairs) = int(OrigArray(6, (j / 3) * 3 + 2));
				Pairs(2, numpairs) = 0;
				if((j / 3) * 3 + 2 != j)
					numpairs++;
			}
		}
		
		//If near
		for(int j = 0; j < OrigVertices; j++)
		{
			x = OrigVertexArray(i, 0) - OrigVertexArray(j, 0);
			y = OrigVertexArray(i, 1) - OrigVertexArray(j, 1);
			z = OrigVertexArray(i, 2) - OrigVertexArray(j, 2);
			dist = sqrt(x * x + y * y + z * z);
			
			if(dist <= 0.05)
			{
				Pairs(0, numpairs) = i;
				Pairs(1, numpairs) = j;
				Pairs(2, numpairs) = 1;
				numpairs++;
			}
		}
	}
	
	//Find where to contract to for each pair
	MatrixXf PairPos(4, OrigVertices * OrigVertices);
	MatrixXf PairQ(4, OrigVertices * OrigVertices * 4);
	MatrixXf PairCost(1, OrigVertices * OrigVertices);
	MatrixXf TempArr(4, 4);
	
	for(int i = 0; i < numpairs; i++)
	{
		//Make Q from pair of vertices
		PairQ.col(i * 4) = QArray.col(Pairs(0, i) * 4) + QArray.col(Pairs(1, i) * 4);
		PairQ.col(i * 4 + 1) = QArray.col(Pairs(0, i) * 4 + 1) + QArray.col(Pairs(1, i) * 4 + 1);
		PairQ.col(i * 4 + 2) = QArray.col(Pairs(0, i) * 4 + 2) + QArray.col(Pairs(1, i) * 4 + 2);
		PairQ.col(i * 4 + 3) = QArray.col(Pairs(0, i) * 4 + 3) + QArray.col(Pairs(1, i) * 4 + 3);
		
		//Minimize error
		TempArr.col(0) = PairQ.col(i * 4);
		TempArr.col(1) = PairQ.col(i * 4 + 1);
		TempArr.col(2) = PairQ.col(i * 4 + 2);
		TempArr.col(3) = PairQ.col(i * 4 + 3);
		TempArr.row(3) = RowVector4f(0, 0, 0, 1);
		
		if(TempArr.determinant() != 0)
			PairPos.col(i) = TempArr.inverse() * Vector4f(0, 0, 0, 1);
		else
		{
			PairPos(0, i) = (OrigVertexArray(Pairs(0, i), 0) / 2.0f) + (OrigVertexArray(Pairs(1, i), 0) / 2.0f);
			PairPos(1, i) = (OrigVertexArray(Pairs(0, i), 1) / 2.0f) + (OrigVertexArray(Pairs(1, i), 1) / 2.0f);
			PairPos(2, i) = (OrigVertexArray(Pairs(0, i), 2) / 2.0f) + (OrigVertexArray(Pairs(1, i), 2) / 2.0f);
			PairPos(3, i) = 1;
		}
		
		TempArr.col(0) = PairQ.col(i * 4);
		TempArr.col(1) = PairQ.col(i * 4 + 1);
		TempArr.col(2) = PairQ.col(i * 4 + 2);
		TempArr.col(3) = PairQ.col(i * 4 + 3);
		PairCost(0, i) = PairPos.col(i).transpose() * TempArr * PairPos.col(i);
	}
	
	//Loop here
	BunnyFaces = OrigFaces;
	BunnyVertices = OrigVertices;
	BunnyArray = OrigArray;
	float cheapest;
	int index;
	while(BunnyFaces > numfaces)
	{
		//Contract cheapest pair
		cheapest = 100;
		index = -1;
		for(int i = 0; i < numpairs; i++)
		{
			if(PairCost(0, i) < cheapest && Pairs(2, i) != -1)
			{
				cheapest = PairCost(0, i);
				index = i;
			}
		}
		
		if(index == -1)
		{
			cout << "Smallest number of faces = " << BunnyFaces << '\n';
			return;
		}
		
		//Replace v2 with v1 in drawn array and set new position
		for(int i = 0; i < BunnyFaces * 3; i++)
		{
			if(BunnyArray(6, i) == Pairs(0, index))
			{
				BunnyArray(0, i) = PairPos(0, index);
				BunnyArray(1, i) = PairPos(1, index);
				BunnyArray(2, i) = PairPos(2, index);
			}
			if(BunnyArray(6, i) == Pairs(1, index))
			{
				BunnyArray(0, i) = PairPos(0, index);
				BunnyArray(1, i) = PairPos(1, index);
				BunnyArray(2, i) = PairPos(2, index);
				BunnyArray(6, i) = float(Pairs(0, index));
			}
		}
		//Replace instances of v2 with v1 in pairs array
		for(int i = 0; i < numpairs; i++)
		{
			if(Pairs(0, i) == Pairs(1, index))
				Pairs(0, i) = Pairs(0, index);
			if(Pairs(1, i) == Pairs(1, index))
				Pairs(1, i) = Pairs(0, index);
			
			if(Pairs(0, i) == Pairs(1, i))
				Pairs(2, i) = -1;
		}
		
		//Remove nonexistent faces
		for(int i = 0; i < BunnyFaces; i++)
		{
			if(BunnyArray(6, i * 3) == BunnyArray(6, i * 3 + 1) || BunnyArray(6, i * 3 + 1) == BunnyArray(6, i * 3 + 2) ||
				BunnyArray(6, i * 3 + 2) == BunnyArray(6, i * 3))
			{
				RemoveFace(i);
				BunnyFaces--;
			}
		}
		
		//Update error of this vertex
		for(int i = 0; i < BunnyFaces; i++)
		{
			v1 << BunnyArray(0, i * 3), BunnyArray(1, i * 3), BunnyArray(2, i * 3);
			v2 << BunnyArray(0, i * 3 + 1), BunnyArray(1, i * 3 + 1), BunnyArray(2, i * 3 + 1);
			v3 << BunnyArray(0, i * 3 + 2), BunnyArray(1, i * 3 + 2), BunnyArray(2, i * 3 + 2);
			normal = CalcNormal(v1, v2, v3);
			
			x = normal[0];
			y = normal[1];
			z = normal[2];
			
			d = -x * BunnyArray(0, i * 3) - y * BunnyArray(1, i * 3) - z * BunnyArray(2, i * 3);
			Norms(0, i) = x;
			Norms(1, i) = y;
			Norms(2, i) = z;
			Norms(3, i) = d;
		}
		
		int temp = Pairs(0, index);
		for(int j = 0; j < BunnyFaces * 3; j++)
		{
			if(BunnyArray(6, j) == temp)
			{
				QArray.col(temp * 4) = QArray.col(temp * 4) + Vector4f(Norms(0, j / 3) * Norms(0, j / 3),
					Norms(0, j / 3) * Norms(1, j / 3), Norms(0, j / 3) * Norms(2, j / 3), Norms(0, j / 3) * Norms(3, j / 3));
				
				QArray.col(temp * 4 + 1) = QArray.col(temp * 4 + 1) + Vector4f(Norms(1, j / 3) * Norms(0, j / 3),
					Norms(1, j / 3) * Norms(1, j / 3), Norms(1, j / 3) * Norms(2, j / 3), Norms(1, j / 3) * Norms(3, j / 3));
				
				QArray.col(temp * 4 + 2) = QArray.col(temp * 4 + 2) + Vector4f(Norms(2, j / 3) * Norms(0, j / 3),
					Norms(2, j / 3) * Norms(1, j / 3), Norms(2, j / 3) * Norms(2, j / 3), Norms(2, j / 3) * Norms(3, j / 3));
					
				QArray.col(temp * 4 + 3) = QArray.col(temp * 4 + 3) + Vector4f(Norms(3, j / 3) * Norms(0, j / 3),
					Norms(3, j / 3) * Norms(1, j / 3), Norms(3, j / 3) * Norms(2, j / 3), Norms(3, j / 3) * Norms(3, j / 3));
			}
		}
		
		//Update costs for pairs that contained either vertex
		for(int i = 0; i < numpairs; i++)
		{
			if((Pairs(0, i) == Pairs(0, index) || Pairs(1, i) == Pairs(0, index)) && Pairs(2, i) != -1)
			{
				PairQ.col(i * 4) = QArray.col(Pairs(0, i) * 4) + QArray.col(Pairs(1, i) * 4);
				PairQ.col(i * 4 + 1) = QArray.col(Pairs(0, i) * 4 + 1) + QArray.col(Pairs(1, i) * 4 + 1);
				PairQ.col(i * 4 + 2) = QArray.col(Pairs(0, i) * 4 + 2) + QArray.col(Pairs(1, i) * 4 + 2);
				PairQ.col(i * 4 + 3) = QArray.col(Pairs(0, i) * 4 + 3) + QArray.col(Pairs(1, i) * 4 + 3);
				
				TempArr.col(0) = PairQ.col(i * 4);
				TempArr.col(1) = PairQ.col(i * 4 + 1);
				TempArr.col(2) = PairQ.col(i * 4 + 2);
				TempArr.col(3) = PairQ.col(i * 4 + 3);
				TempArr.row(3) = RowVector4f(0, 0, 0, 1);
				
				if(TempArr.determinant() != 0)
					PairPos.col(i) = TempArr.inverse() * Vector4f(0, 0, 0, 1);
				else
				{
					PairPos(0, i) = (OrigVertexArray(Pairs(0, i), 0) / 2.0f) + (OrigVertexArray(Pairs(1, i), 0) / 2.0f);
					PairPos(1, i) = (OrigVertexArray(Pairs(0, i), 1) / 2.0f) + (OrigVertexArray(Pairs(1, i), 1) / 2.0f);
					PairPos(2, i) = (OrigVertexArray(Pairs(0, i), 2) / 2.0f) + (OrigVertexArray(Pairs(1, i), 2) / 2.0f);
					PairPos(3, i) = 1;
				}
				
				TempArr.col(0) = PairQ.col(i * 4);
				TempArr.col(1) = PairQ.col(i * 4 + 1);
				TempArr.col(2) = PairQ.col(i * 4 + 2);
				TempArr.col(3) = PairQ.col(i * 4 + 3);
				PairCost(0, i) = PairPos.col(i).transpose() * TempArr * PairPos.col(i);
			}
		}
		
		BunnyVertices--;
	}
	
	
	//After contraction----
	
	//Recalculate normals
	total = RowVector3f(0, 0, 0);
	
	for(int i = 0; i < BunnyFaces; i++)
	{
		v1 << BunnyArray(0, i * 3), BunnyArray(1, i * 3), BunnyArray(2, i * 3);
		v2 << BunnyArray(0, i * 3 + 1), BunnyArray(1, i * 3 + 1), BunnyArray(2, i * 3 + 1);
		v3 << BunnyArray(0, i * 3 + 2), BunnyArray(1, i * 3 + 2), BunnyArray(2, i * 3 + 2);
		normal = CalcNormal(v1, v2, v3);
		
		for(int j = 0; j < BunnyFaces * 3; j++)
		{
			if(BunnyArray(6, i * 3) == BunnyArray(6, j))
			{
				BunnyArray(3, j) += normal[0];
				BunnyArray(4, j) += normal[1];
				BunnyArray(5, j) += normal[2];
			}
			
			if(BunnyArray(6, i * 3 + 1) == BunnyArray(6, j))
			{
				BunnyArray(3, j) += normal[0];
				BunnyArray(4, j) += normal[1];
				BunnyArray(5, j) += normal[2];
			}
				
			if(BunnyArray(6, i * 3 + 2) == BunnyArray(6, j))
			{
				BunnyArray(3, j) += normal[0];
				BunnyArray(4, j) += normal[1];
				BunnyArray(5, j) += normal[2];
			}
		}
	}
	for(int i = 0; i < BunnyFaces * 3; i++)
	{
		total = RowVector3f(BunnyArray(3, i), BunnyArray(4, i), BunnyArray(5, i));
		total.normalize();
		BunnyArray(3, i) = total[0];
		BunnyArray(4, i) = total[1];
		BunnyArray(5, i) = total[2];
	}
	
	return;
}

//------------------------------------


//For when the window is resized
void resize_callback(GLFWwindow* window, int width, int height)
{
	if(width >= height)
		glViewport((width - height) / 2, 0, height, height);
	else
		glViewport(0, (height - width) / 2, width, width);
	
	return;
}


//For when a key is pressed
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if(action == GLFW_RELEASE)
		return;
	
	switch (key)
	{
		case GLFW_KEY_Y:
			Contraction();
			break;
		
		default:
			break;
	}
	
	//Translate using w, a, s, d, q, and e
	switch (key)
	{
		case GLFW_KEY_W:
			BunnyData(9, 0) += 0.1f;
			break;
			
		case GLFW_KEY_S:
			BunnyData(9, 0) -= 0.1f;
			break;
		
		case GLFW_KEY_A:
			BunnyData(8, 0) -= 0.1f;
			break;
			
		case GLFW_KEY_D:
			BunnyData(8, 0) += 0.1f;
			break;
			
		case GLFW_KEY_Q:
			BunnyData(10, 0) -= 0.1f;
			break;
			
		case GLFW_KEY_E:
			BunnyData(10, 0) += 0.1f;
			break;
		
		default:
			break;
	}
	
	//Scaling using n and m
	switch (key)
	{
		case GLFW_KEY_N:
			BunnyData(4, 0) = BunnyData(4, 0) * 0.8f;
			break;
			
		case GLFW_KEY_M:
			BunnyData(4, 0) = BunnyData(4, 0) * 1.2f;
			break;
		
		default:
			break;
	}
	
	//Rotate using i, j, k, l, u, and o
	switch (key)
	{
		case GLFW_KEY_I:
			BunnyData(5, 0) += 31.415926f / 180.0f;
			break;
			
		case GLFW_KEY_K:
			BunnyData(5, 0) -= 31.415926f / 180.0f;
			break;
		
		case GLFW_KEY_J:
			BunnyData(6, 0) -= 31.415926f / 180.0f;
			break;
			
		case GLFW_KEY_L:
			BunnyData(6, 0) += 31.415926f / 180.0f;
			break;
			
		case GLFW_KEY_U:
			BunnyData(7, 0) -= 31.415926f / 180.0f;
			break;
			
		case GLFW_KEY_O:
			BunnyData(7, 0) += 31.415926f / 180.0f;
			break;
		
		default:
			break;
	}
	
	//Rotate screen around y-axis with up, down, left, and right keys
	switch (key)
	{
		case GLFW_KEY_UP:
			ScreenRotation[0] -= 31.415926f / 180.0f;
			glUniform2f(ViewTrans3dx, cos(ScreenRotation[0]), sin(ScreenRotation[0]));
			break;
			
		case GLFW_KEY_DOWN:
			ScreenRotation[0] += 31.415926f / 180.0f;
			glUniform2f(ViewTrans3dx, cos(ScreenRotation[0]), sin(ScreenRotation[0]));
			break;
			
		case GLFW_KEY_LEFT:
			ScreenRotation[1] -= 31.415926f / 180.0f;
			glUniform2f(ViewTrans3dy, cos(ScreenRotation[1]), sin(ScreenRotation[1]));
			break;
			
		case GLFW_KEY_RIGHT:
			ScreenRotation[1] += 31.415926f / 180.0f;
			glUniform2f(ViewTrans3dy, cos(ScreenRotation[1]), sin(ScreenRotation[1]));
			break;
		
		default:
			break;
	}
	
	//Set shading mode
	switch(key)
	{
		case GLFW_KEY_Z:
			BunnyData(3, 0) = 0.0f;
			break;
		
		case GLFW_KEY_X:
			BunnyData(3, 0) = 1.0f;
			break;
			
		case GLFW_KEY_C:
			BunnyData(3, 0) = 2.0f;
			break;
	}
	
	//Export as OBJ file
	ofstream write;
	Matrix<int, Dynamic, Dynamic> VertexCheck;
	VertexCheck = Matrix<int, Dynamic, Dynamic>::Zero(1, OrigVertices);
	switch(key)
	{
		case GLFW_KEY_P:
			write.open("../data/export.obj");
			
			for(int j = 0; j < OrigVertices; j++)
			{
				for(int i = 0; i < BunnyFaces * 3; i++)
				{
					if(BunnyArray(6, i) == j)
					{
						write << "v " << BunnyArray(0, i) << ' ';
						write << BunnyArray(1, i) << ' ' << BunnyArray(2, i) << '\n';
						
						VertexCheck(0, j) = 1;
						break;
					}
				}
			}
			
			
			int counter;
			for(int i = 0; i < BunnyFaces; i++)
			{
				counter = 0;
				for(int j = 0; j < BunnyArray(6, i * 3); j++)
				{
					if(VertexCheck(0, j) == 1)
						counter++;
				}
				write << "f " << counter + 1 << ' ';
				
				counter = 0;
				for(int j = 0; j < BunnyArray(6, i * 3 + 1); j++)
				{
					if(VertexCheck(0, j) == 1)
						counter++;
				}
				write << counter + 1 << ' ';
				
				counter = 0;
				for(int j = 0; j < BunnyArray(6, i * 3); j++)
				{
					if(VertexCheck(0, j) == 1)
						counter++;
				}
				write << counter + 1 << '\n';
			}
			write << "end\n";
			write.close();
	}
	
	return;
}


//MAIN FUNCTIONS
int main(void)
{
	Program3dRun();
	return 0;
}

//The 3d version of the above program
void Program3dRun()
{
	//Lame required stuff
	GLFWwindow* window;

	//Initialize the library
	if (!glfwInit())
		return;

	//Activate supersampling
	glfwWindowHint(GLFW_SAMPLES, 8);

	//Ensure that we get at least a 3.2 context
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);

	//On apple we have to load a core profile with forward compatibility
	#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	#endif

	//Create a windowed mode window and its OpenGL context
	window = glfwCreateWindow(640, 640, "Rasterization 3D", NULL, NULL);
	window3d = window;
	if (!window)
	{
		glfwTerminate();
		return;
	}

	//Make the window's context current
	glfwMakeContextCurrent(window);

	#ifndef __APPLE__
	glewExperimental = true;
	GLenum err = glewInit();
	if(GLEW_OK != err)
	{
		/*Problem: glewInit failed, something is seriously wrong.*/
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
	}
	glGetError(); //pull and savely ignonre unhandled errors like GL_INVALID_ENUM
	fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));
	#endif

	//End Required Stuff
	//------------------
	
	
	//Initialize the VAO
	glGenVertexArrays(1, &VAO);
	glBindVertexArray(VAO);

	//Initialize the VBO with the vertices data
	glGenBuffers(1, &VBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	
	
	const GLchar* vertex_shader =
			"#version 150 core\n"
				"in vec3 position;"
				"in vec3 normal;"
				
				"uniform vec2 viewtrx;"
				"uniform vec2 viewtry;"
				"uniform vec2 viewtrz;"
				"uniform vec3 objectscale = vec3(1.0f, 1.0f, 1.0f);"
				"uniform vec2 objrotx;"
				"uniform vec2 objroty;"
				"uniform vec2 objrotz;"
				"uniform vec3 objecttranslation = vec3(0.0f, 0.0f, 0.0f);"
				"uniform vec3 barycenter = vec3(0.0f, 0.0f, 0.0f);"
				"uniform vec3 lightsource;"
				"uniform vec2 windowdim;"
				
				"out vec4 phongResult;"
				
				"void main()"
				"{"
				"	mat4 rotate;"
				"	mat4 viewrotate;"
				"	vec4 hposition = vec4(position, 1.0f);"
				"	vec4 hnormal = vec4(normal, 0.0f);"
				"	vec3 barydist = hposition.xyz - barycenter;"
				"	float color;"
				
				//Rotation matrix
				"	vec4 matrix1 = vec4(objrotz[0] * objroty[0], objrotz[1] * objroty[0], -objroty[1], 0);"
				"	vec4 matrix2 = vec4(objrotz[0] * objroty[1] * objrotx[1] - objrotz[1] * objrotx[0],"
				"						objrotz[1] * objroty[1] * objrotx[1] + objrotz[0] * objrotx[0], objroty[0] * objrotx[1], 0);"
				"	vec4 matrix3 = vec4(objrotz[0] * objroty[1] * objrotx[0] + objrotz[1] * objrotx[1],"
				"						objrotz[1] * objroty[1] * objrotx[0] - objrotz[0] * objrotx[1], objroty[0] * objrotx[0], 0);"
				"	vec4 matrix4 = vec4(0, 0, 0, 1);"
				"	rotate[0] =  matrix1;"
				"	rotate[1] =  matrix2;"
				"	rotate[2] =  matrix3;"
				"	rotate[3] =  matrix4;"
				
				"	vec3 baryscale = barydist * objectscale + barycenter;"
				"	hposition = vec4(baryscale, hposition[3]);" //Scales vertices
				"	barydist = hposition.xyz - barycenter;"
				"	hposition = rotate * vec4(barydist, 1.0f) + vec4(barycenter, 0.0f);" //Rotates vertices
				
				"	hposition = vec4(objecttranslation + hposition.xyz, hposition[3]);" //Translates vertices
				
				//Screen Rotation and resizing
				"	vec4 rotmat1 = vec4(viewtrz[0] * viewtry[0], viewtrz[1] * viewtry[0], -viewtry[1], 0);"
				"	vec4 rotmat2 = vec4(viewtrz[0] * viewtry[1] * viewtrx[1] - viewtrz[1] * viewtrx[0],"
				"					viewtrz[1] * viewtry[1] * viewtrx[1] + viewtrz[0] * viewtrx[0], viewtry[0] * viewtrx[1], 0);"
				"	vec4 rotmat3 = vec4(viewtrz[0] * viewtry[1] * viewtrx[0] + viewtrz[1] * viewtrx[1],"
				"					viewtrz[1] * viewtry[1] * viewtrx[0] - viewtrz[0] * viewtrx[1], viewtry[0] * viewtrx[0], 0);"
				"	vec4 rotmat4 = vec4(0, 0, 0, 1);"
				"	viewrotate[0] =  rotmat1;"
				"	viewrotate[1] =  rotmat2;"
				"	viewrotate[2] =  rotmat3;"
				"	viewrotate[3] =  rotmat4;"
				
				"	hposition = viewrotate * hposition;"
				"	hposition = vec4(hposition.xy * windowdim, hposition[2], hposition[3]);"
				
				//Lighting calculations
				"	vec4 lightray = vec4((position - lightsource), 0.0f);"
				"	hnormal = rotate * hnormal;"
				"	color = dot(hnormal, lightray);"
				"	phongResult = vec4(color, color, color, 0.0f) * 0.3;"
				
				"	gl_Position = vec4(hposition);"
				"}";
	const GLchar* fragment_shader =
			"#version 150 core\n"
				"in vec4 phongResult;"
				"out vec4 outColor;"
				
				"uniform vec4 selectcolor;" //Changes color of selected object
				"uniform vec3 vcolor;"
				"uniform float phongshade;"
				
				"void main()"
				"{"
				"	vec4 finalcolor = vec4(0.2f, 0.2f, 0.2f, 1.0f);"
				"	finalcolor = vec4(vcolor, 1.0f) + phongshade * phongResult;"
				"	finalcolor = finalcolor + selectcolor;"
				"	outColor = vec4(finalcolor);"
				"}";
	
	//Vertex Shader
	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, &vertex_shader, NULL);
	glCompileShader(vertexShader);
	
	//Fragment Shader
	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShader, 1, &fragment_shader, NULL);
	glCompileShader(fragmentShader);
	
	//Program Creation
	GLuint Program3d = glCreateProgram();
	glAttachShader(Program3d, vertexShader);
	glAttachShader(Program3d, fragmentShader);
	glBindFragDataLocation(Program3d, 0, "outColor");
	glLinkProgram(Program3d);
	
	//Linking Inputs
	GLint posAttrib = glGetAttribLocation(Program3d, "position");
	glEnableVertexAttribArray(posAttrib);
	glVertexAttribPointer(posAttrib, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), 0);
	
	GLint normalAttrib = glGetAttribLocation(Program3d, "normal");
	glEnableVertexAttribArray(normalAttrib);
	glVertexAttribPointer(normalAttrib, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)(3 * sizeof(float)));
	
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	
	glUseProgram(Program3d);
	
	//Uniforms
	ViewTrans3dx = glGetUniformLocation(Program3d, "viewtrx");
	ViewTrans3dy = glGetUniformLocation(Program3d, "viewtry");
	ViewTrans3dz = glGetUniformLocation(Program3d, "viewtrz");
	Scale3d = glGetUniformLocation(Program3d, "objectscale");
	Rotation3dx = glGetUniformLocation(Program3d, "objrotx");
	Rotation3dy = glGetUniformLocation(Program3d, "objroty");
	Rotation3dz = glGetUniformLocation(Program3d, "objrotz");
	Transl3d = glGetUniformLocation(Program3d, "objecttranslation");
	SelectColor = glGetUniformLocation(Program3d, "selectcolor");
	Barycenter3d = glGetUniformLocation(Program3d, "barycenter");
	VertexColor = glGetUniformLocation(Program3d, "vcolor");
	PhongShade = glGetUniformLocation(Program3d, "phongshade");
	LightSource = glGetUniformLocation(Program3d, "lightsource");
	WindowSize = glGetUniformLocation(Program3d, "windowdim");
	
	//Register the keyboard callback
	glfwSetKeyCallback(window, key_callback);
	
	//Register screen resize callback
	glfwSetWindowSizeCallback(window, resize_callback);
	
	
	//MAIN FUNCTIONS
	
	//Populate CubeArray, BunnyArray, and BumpyCubeArray
	string trashstring;
	int trashint;
	int temp1, temp2, temp3;
	ifstream read;
	
	
	//Bunny is scaled by 5 times
	read.open("../data/bunny.off");
	
	read >> trashstring;
	read >> BunnyVertices >> BunnyFaces >> trashint;
	OrigVertexArray = Matrix<float, Dynamic, Dynamic>::Zero(BunnyVertices, 3);
	BunnyArray = Matrix<float, Dynamic, Dynamic>::Zero(7, BunnyFaces * 3);
	
	float totalx = 0, totaly = 0, totalz = 0;
	for(int i = 0; i < BunnyVertices; i++)
	{
		read >> OrigVertexArray(i, 0) >> OrigVertexArray(i, 1) >> OrigVertexArray(i, 2);
		OrigVertexArray(i, 0) = OrigVertexArray(i, 0) * 5.0f;
		OrigVertexArray(i, 1) = OrigVertexArray(i, 1) * 5.0f;
		OrigVertexArray(i, 2) = OrigVertexArray(i, 2) * 5.0f;
		
		totalx += OrigVertexArray(i, 0);
		totaly += OrigVertexArray(i, 1);
		totalz += OrigVertexArray(i, 2);
	}
	
	for(int i = 0; i < BunnyFaces; i++)
	{
		read >> trashint >> temp1 >> temp2 >> temp3;
		for(int j = 0; j < 3; j++)
		{
			BunnyArray(j, i * 3) = OrigVertexArray(temp1, j);
			BunnyArray(j, i * 3 + 1) = OrigVertexArray(temp2, j);
			BunnyArray(j, i * 3 + 2) = OrigVertexArray(temp3, j);
		}
		
		BunnyArray(6, i * 3) = float(temp1);
		BunnyArray(6, i * 3 + 1) = float(temp2);
		BunnyArray(6, i * 3 + 2) = float(temp3);
	}
	
	read.close();
	
	
	//Calculate vertex normals
	Vector3f v1, v2, v3, normal, total;

	for(int i = 0; i < BunnyFaces; i++)
	{
		v1 << BunnyArray(0, i * 3), BunnyArray(1, i * 3), BunnyArray(2, i * 3);
		v2 << BunnyArray(0, i * 3 + 1), BunnyArray(1, i * 3 + 1), BunnyArray(2, i * 3 + 1);
		v3 << BunnyArray(0, i * 3 + 2), BunnyArray(1, i * 3 + 2), BunnyArray(2, i * 3 + 2);
		normal = CalcNormal(v1, v2, v3);
		
		for(int j = 0; j < BunnyFaces * 3; j++)
		{
			if(BunnyArray(6, i * 3) == BunnyArray(6, j))
			{
				BunnyArray(3, j) += normal[0];
				BunnyArray(4, j) += normal[1];
				BunnyArray(5, j) += normal[2];
			}
			
			if(BunnyArray(6, i * 3 + 1) == BunnyArray(6, j))
			{
				BunnyArray(3, j) += normal[0];
				BunnyArray(4, j) += normal[1];
				BunnyArray(5, j) += normal[2];
			}
				
			if(BunnyArray(6, i * 3 + 2) == BunnyArray(6, j))
			{
				BunnyArray(3, j) += normal[0];
				BunnyArray(4, j) += normal[1];
				BunnyArray(5, j) += normal[2];
			}
		}
	}
	for(int i = 0; i < BunnyFaces * 3; i++)
	{
		total = RowVector3f(BunnyArray(3, i), BunnyArray(4, i), BunnyArray(5, i));
		total.normalize();
		BunnyArray(3, i) = total[0];
		BunnyArray(4, i) = total[1];
		BunnyArray(5, i) = total[2];
	}
	
	OrigArray = BunnyArray;
	OrigFaces = BunnyFaces;
	OrigVertices = BunnyVertices;
	
	//Populate data fields
	totalx = totalx / BunnyVertices;
	totaly = totaly / BunnyVertices;
	totalz = totalz / BunnyVertices;

	BunnyData.col(0) <<  totalx, totaly, totalz, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
	
	ScreenRotation << 0.0f, 0.0f, 0.0f;
	glUniform2f(ViewTrans3dx, cos(ScreenRotation[0]), sin(ScreenRotation[0]));
	glUniform2f(ViewTrans3dy, cos(ScreenRotation[1]), sin(ScreenRotation[1]));
	glUniform2f(ViewTrans3dz, cos(ScreenRotation[2]), sin(ScreenRotation[2]));
	glUniform1f(PhongShade, 0.0f);
	glUniform3f(LightSource, -2.0f, 2.0f, -2.0f);
	glUniform2f(WindowSize, 1.0f, 1.0f);
	glUniform4f(SelectColor, 0.0f, 0.0f, 0.0f, 0.0f);
	
	//MAIN LOOP
	while (!glfwWindowShouldClose(window))
	{
		//Bind program
		glUseProgram(Program3d);

		//Clear the framebuffer
		glClearColor(0.95f, 0.95f, 0.95f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		
		
		//Draw Bunnies
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * OrigFaces * 3 * 7, BunnyArray.data(), GL_DYNAMIC_DRAW);
		
		//Barycenter [0-2]
		glUniform3f(Barycenter3d, BunnyData(0, 0), BunnyData(1, 0), BunnyData(2, 0));
		//Scaling [4]
		glUniform3f(Scale3d, BunnyData(4, 0), BunnyData(4, 0), BunnyData(4, 0));
		//Rotation [5-7]
		glUniform2f(Rotation3dx, cos(BunnyData(5, 0)), sin(BunnyData(5, 0)));
		glUniform2f(Rotation3dy, cos(BunnyData(6, 0)), sin(BunnyData(6, 0)));
		glUniform2f(Rotation3dz, cos(BunnyData(7, 0)), sin(BunnyData(7, 0)));
		//Translation [8-10]
		glUniform3f(Transl3d, BunnyData(8, 0), BunnyData(9, 0), BunnyData(10, 0));
		
		//Shading type [3]
		//Wire frame
		if(BunnyData(3, 0) == 0.0f)
		{
			glUniform3f(VertexColor, 0.0f, 0.0f, 0.0f);
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glDrawArrays(GL_TRIANGLES, 0, BunnyFaces * 3);
		}
		//Flat shading
		else if(BunnyData(3, 0) == 1.0f)
		{
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			
			for(int j = 0; j < BunnyFaces; j++)
			{
				v1 << BunnyArray(0, j * 3), BunnyArray(1, j * 3), BunnyArray(2, j * 3);
				v2 << BunnyArray(0, j * 3 + 1), BunnyArray(1, j * 3 + 1), BunnyArray(2, j * 3 + 1);
				v3 << BunnyArray(0, j * 3 + 2), BunnyArray(1, j * 3 + 2), BunnyArray(2, j * 3 + 2);
				
				FlatShading(v1, v2, v3); //Will update uniform for this triangle
				glDrawArrays(GL_TRIANGLES, j * 3, 3);
			}
			
			glUniform3f(VertexColor, 0.0f, 0.0f, 0.0f);
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glDrawArrays(GL_TRIANGLES, 0, BunnyFaces * 3);
		}
		//Phong shading
		else if(BunnyData(3, 0) == 2.0f)
		{
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glUniform3f(VertexColor, 0.0f, 0.0f, 0.0f);
			glUniform1f(PhongShade, 1.0f);
			glDrawArrays(GL_TRIANGLES, 0, BunnyFaces * 3);
			glUniform1f(PhongShade, 0.0f);
		}
		
		
		check_gl_error();
		//Swap front and back buffers
		glfwSwapBuffers(window);
		
		//Poll for and process events
		glfwPollEvents();
	}
	
	//Ending Stuff
	//------------

	// Deallocate opengl memory
	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
	glDeleteProgram(Program3d);
	
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);
	glfwTerminate();
	
	return;
}