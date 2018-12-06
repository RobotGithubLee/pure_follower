
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "BSping.h"




/*****************************************************************************\
* FUNCTION:			getDistance
*
* DESCRIPTION:      getDistance
\*****************************************************************************/
float getDistance(Point3D StartPt, Point3D EndPt)
{
  float Dist = (EndPt.X - StartPt.X) * (EndPt.X - StartPt.X)
	      + (EndPt.Y - StartPt.Y) * (EndPt.Y - StartPt.Y)
		  + (EndPt.Z - StartPt.Z) * (EndPt.Z - StartPt.Z);
  return sqrt(Dist);
}

/*****************************************************************************\
* FUNCTION:			 getFitPointU
* DESCRIPTION :      get U of FitPoint
*
* INPUT:             fitPoint[]  need fitting piont
                     fitNum      the num of fitting point
                     flag        0 chord lengh parameterize, 1 center parameterize

* OUTPUT£º           uk[]        u of fitting point
*****************************************************************************/

void getFitPointU(Point3D fitPoint[], int fitNum, int flag, float uk[])
{
	int i;
	float sumChordLen = 0;
	float chordLen[MaxFitPointNum] = { 0 };

	uk[0] = 0;

	if (flag == 0)
	{
		for (i = 0; i<fitNum - 1; i++)
		{
			chordLen[i] = getDistance(fitPoint[i], fitPoint[i + 1]);
			sumChordLen += chordLen[i];
		}

		for (i = 0; i < fitNum - 2; i++)
		{
			uk[i + 1] = uk[i] + chordLen[i] / sumChordLen;
		}
	}
	else
	{
		for (i = 0; i<fitNum - 1; i++)
		{
			chordLen[i] = sqrt(getDistance(fitPoint[i], fitPoint[i + 1]));
			sumChordLen += chordLen[i];
		}

		for (i = 0; i<fitNum - 2; i++)
		{
			uk[i + 1] = uk[i] + chordLen[i] / sumChordLen;
		}
	}

	uk[fitNum - 1] = 1;

}


/*****************************************************************************\
* FUNCTION:			 getVectorU
* DESCRIPTION :      get Vector U
*
* INPUT:             fitNum      the num of fitting point
                     uk[]        u of fitting point

* OUTPUT£º           U[]         node vector
*****************************************************************************/

void getVectorU(int fitNum, float uk[], float U[])
{
	int i, j;
	float temp;

	for (i = 0; i <= P; i++)
	{
		U[i] = 0;
	}

	for (i = P + 1; i <fitNum; i++)
	{
		temp = 0;

		for (j = 0; j < P; j++)
		{
			temp += uk[i - P + j];
		}

		U[i] = temp / P;

	}

	for (i = fitNum; i < fitNum + P + 1; i++)
	{
		U[i] = 1;
	}
}



/*****************************************************************************\
* FUNCTION:			findSpan
* DESCRIPTION:      find span u in U[]
*
* INPUT:            u          u of fitting point
                    U[]        node vector
                    fitNum     number of fitting point

* RETURN:           Span of u in U[]
\*****************************************************************************/
int findSpan(float u, float U[], int fitNum)
{
	int low, mid, high;

	if (u == U[fitNum])
		return fitNum - 1;

	low = P;
	high = fitNum;
	mid = (low + high) / 2;
	while (u < U[mid] || u >= U[mid + 1])
	{
		if (u > U[mid])
			low = mid;
		else
			high = mid;
		mid = (low + high) / 2;
	}
	return mid;
}



/*****************************************************************************\
* FUNCTION:			getBasisFuns
* DESCRIPTION:      get Basis Funs
*
* INPUT:            i         the sapn of u
                    u         u of fitting point
                    U[]       node vector

* OUTPUT:           BasiFuns
\*****************************************************************************/
void getBasisFuns(int i, float u, float U[], float N[])
{
	int j, k;
	float left[P + 1] = { 0 };
	float right[P + 1] = { 0 };
	float saved, temp;

	N[0] = 1.0;
	for (j = 1; j <= P; j++)
	{
		left[j] = u - U[i + 1 - j];
		right[j] = U[i + j] - u;
		saved = 0.0;
		for (k = 0; k<j; k++)
		{
			if ((right[k + 1] + left[j - k]) == 0)
				break;
			temp = N[k] / (right[k + 1] + left[j - k]);
			N[k] = saved + right[k + 1] * temp;
			saved = left[j - k] * temp;
		}
		N[j] = saved;
	}
}


/*****************************************************************************\
* FUNCTION:			getDersBasisFuns
* DESCRIPTION:      get the ders of basis function
*
* INPUT:            u         u of fitting point
                    i         Span of fitting point
                    fitNim    Num of fitting points
                    n         Order of basis function
                    U[]       node vector

* OUTPUT:           ders[][P+1]     ders of basis function
\*****************************************************************************/
void getDersBasisFuns(float u, int i, int fitNum, int n, float U[], float ders[][P + 1])
{
	int j, r, k;

	long rk, pk, j1, j2;
	long  s1, s2;
	float d, temp, saved;


	float left[P + 1];
	float right[P + 1];
	float ndu[P + 1][P + 1];
	float a[P + 1][P + 1];

	i = findSpan(u, U, fitNum);


	ndu[0][0] = 1.0;
	for (j = 1; j <= P; j++)
	{
		left[j] = u - U[i + 1 - j];
		right[j] = U[i + j] - u;
		saved = 0.0;
		for (r = 0; r < j; r++)
		{
			//Low triangular matrix save difference between nodes
			ndu[j][r] = right[r + 1] + left[j - r];
			temp = ndu[r][j - 1] / ndu[j][r];

			//Upper triangular matrix save basis function
			ndu[r][j] = saved + right[r + 1] * temp;
			saved = left[j - r] * temp;
		}
		ndu[j][j] = saved;
	}

	//save basis function
	for (j = 0; j <= P; j++)
	{
		ders[0][j] = ndu[j][P];
	}

	// ders of basis function from 1 to n
	for (r = 0; r <= P; r++)
	{
		s1 = 0;
		s2 = 1;

		a[0][0] = 1.0;

		for (k = 1; k <= n; k++)
		{
			d = 0.0;
			rk = r - k;
			pk = P - k;
			if (r >= k)
			{
				a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
				d = a[s2][0] * ndu[rk][pk];
			}
			if (rk >= -1)
			{
				j1 = 1;
			}
			else
			{
				j1 = -rk;
			}
			if (r - 1 <= pk)
			{
				j2 = k - 1;
			}
			else
			{
				j2 = P - r;
			}

			for (j = j1; j <= j2; j++)
			{
				a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
				d += a[s2][j] * ndu[rk + j][pk];
			}
			if (r <= pk)
			{
				a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
				d += a[s2][k] * ndu[r][pk];
			}

			ders[k][r] = d;
			j = s1;
			s1 = s2;
			s2 = j;
		}
	}

	r = P;
	for (k = 1; k <= n; k++)
	{
		for (j = 0; j <= P; j++)
		{
			ders[k][j] *= r;
		}
		r *= (P - k);
	}

}


/*****************************************************************************\
* FUNCTION:			getCurvePoint
* DESCRIPTION:      get the Curve Point
*
* INPUT:            u         u of fitting point
                    fitNim    Num of fitting points
                    U[]       node vector
					fitPoint[]  fit Point
* OUTPUT:           C        ders of basis function
\*****************************************************************************/
Point3D getCurvePoint(float u, int fitNum, float U[], Point3D fitPoint[])
{
	int i, span;
	float N[MaxFitPointNum];
        Point3D C;
	span = findSpan(u, U, fitNum);
	getBasisFuns(span, u, U, N);

	C.X = 0.0;	C.Y = 0.0; C.Z = 0.0;
	for (i = 0; i <= P; i++)
	{
		C.X += N[i] * fitPoint[i + span - P].X;
		C.Y += N[i] * fitPoint[i + span - P].Y;
		C.Z += N[i] * fitPoint[i + span - P].Z;
	}
        return C;
} 


void getCurveDerivsAlg(float u, int fitNum, float U[], Point3D fitPoint[], Point3D CK[])
{
	int k, j, span;
	float nders[P + 1][P + 1];
	span= findSpan(u, U, fitNum);
	getDersBasisFuns(u, span, fitNum, P, U, nders);
	for (k = 0; k < 3; k++)
	{
		CK[k].X = 0.0;
		CK[k].Y = 0.0;
		CK[k].Z = 0.0;
		for (j = 0; j <= P; j++)
		{
			CK[k].X += nders[k][j] * fitPoint[span - P + j].X;
			CK[k].Y += nders[k][j] * fitPoint[span - P + j].Y;
			CK[k].Z += nders[k][j] * fitPoint[span - P + j].Z;
		}
	}
}
