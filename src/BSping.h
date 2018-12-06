#pragma once

#define MaxFitPointNum 10000    //Max count of fitting Point
#define P 3                   //Order of Nurbs

typedef struct
{
  float X;
  float Y;
  float Z;
} Point3D;

float getDistance(Point3D StartPt, Point3D EndPt);

void getFitPointU(Point3D fitPoint[], int fitNum, int flag, float uk[]);

void getVectorU(int fitNum, float uk[], float U[]);

int findSpan(float u, float U[], int fitNum);

void getBasisFuns(int i, float u, float U[], float N[]);

void getDersBasisFuns(float u, int i, int fitNum, int n, float U[], float ders[][P + 1]);

Point3D getCurvePoint(float u, int fitNum, float U[], Point3D fitPoint[]);

void getCurveDerivsAlg(float u, int fitNum, float U[], Point3D fitPoint[], Point3D CK[]);
