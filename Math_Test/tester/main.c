#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define MIN(x,y) ( (x) < (y) ? (x) : (y) )
#define MAX(x,y) ((x)>(y)?(x):(y))
#define SIGN(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define dim 64

float w[dim];
float v[dim][dim];
float d[dim];
float e[dim];

float B_data[dim][dim];

int hshld(float a[dim][dim], int n, float d[dim], float e[dim]);
int fill_array(void);
int print_matrix(float array[dim][dim], int m, int n);
int fill_array(void);
int covariance_fast(float array[dim][dim], int m, int n);


int hshld(float a[dim][dim], int n, float d[dim], float e[dim])
{
	int l, k, j, i;
	float scale, hh, h, g, f;
	for(i=(n-1);i>=1;i--)
	{
		l=i-1;
		h=scale=0.0;
		if(l>0)
		{
			for(k=0;k<l;k++)
			{
				scale += fabs(a[i][k]);
			}
			//arm_abs_f32(&(a[i]), &(a[i]), (uint32_t)l);

			if(scale==0.0)
			{
				//skip transformation
				e[i]=a[i][l];
			}
			else
			{
				for(k=0;k<l;k++)
				{
					a[i][k] /= scale;	//use scaled a's for transformation
					h += a[i][k] * a[i][k];	//form sigma in h
				}
				f = a[i][l];
				g = (f >= 0.0 ? -sqrt(h) : sqrt(h));
				e[i] = scale*g;
				h-=f*g;	//now h = 1/2 abs(u)^2
				a[i][l] = f-g; //store u in ith row of a
				f=0.0;
				for(j=0; j<l; j++)
				{
					//find eigenvectors in the next line
					a[j][i]=a[i][j]/h; //store u/h in ith column of a
					g=0.0; //form an element of A *. u in g
					for (k=0; k<j; k++)
					{
						g+= a[j][k]*a[i][k];
					}
					for (k=j; k<l; k++)
					{
						g+= a[k][j]*a[i][k];
					}
					e[j] = g/h; //form element of p in temporarily unused element of e
					f+= e[j]*a[i][j];
				}
				hh=f/(h+h); //form k = (ut *. p) / 2h
				for(j=0; j<l; j++) //form q and store in e overwriting p
				{
					f=a[i][j];
					e[j]=g=e[j]-hh*f;
					for(k=0;k<j;k++)
					{	//reduce a' = a - q *. ut - u *. qt
						a[j][k] -= (f*e[k]+ g*a[i][k]);
					}
				}
			}
		}else
		{
			e[i] = a[i][l];
		}
		d[i] = h;
	}
	//find eigenvectors next statement
	d[0]=0.0;
	e[0]=0.0;
	//find eigenvectors
	for (i=0; i<n; i++)
	{
		l=i; //begin accumulation of transformation matrices
		if(d[i])
		{
			for(j=0; j<l; j++)
			{
				g = 0.0;
				for(k=0; k<l; k++) //use u and u / h , stored in a, to form p *. q
				{
					g += a[i][k] * a[k][j];
				}
				for(k=0; k<l; k++)
				{
					a[k][j] -= g*a[k][i];
				}
			}
		}
		d[i]=a[i][i]; //eigenvalues
		a[i][i] = 1.0; //reset row and column of a to identity for next iteration
		for(j=0; j<l; j++) a[j][i] = a[i][j] = 0.0;
	}
	print_matrix(a, dim, dim);
}

int fill_array(void)
{
	int m, n;
	for(m=0; m<dim; m++)
	{
		for(n=0; n<dim; n++)
		{
			B_data[n][m] = cos(((float)n/(m+1)) * M_PI);
		}
	}
	return 0;
}

int print_matrix(float array[dim][dim], int m, int n)
{
	int i, j;
	for(i = 0; i < n; i++)
	{
		for(j = 0; j < m; j++)
		{
			printf("%f,\t", array[i][j]);
		}
	printf("\n");
	}
}

int covariance_fast(float array[dim][dim], int m, int n)
{
	int i, j, k;
	float output[n][n];
	k = 0;
		for(i=0; i<n; i++) //initialize output
	{
		for(j=0; j<m; j++)
		{
			output[i][j] = 0.0;
		}
	}
	for(k=0; k<n; k++)
	{
		for(i=0; i<n; i++)
		{
			for(j=i; j<m; j++)
			{
				output[j][i] = output[i][j] += (array[i][k] * array[j][k]);
			}
		}
	}
	print_matrix(output, n, n);
	for(i=0; i<n; i++)
	{
		for(j=i; j<n; j++)
		{
			array[i][j] = array[j][i] = output[i][j];
		}
	}
	printf("\n");
	print_matrix(array, n, n);
}

int main(void)
{
	if(!fill_array())
	{
		printf("A: ");
		print_matrix(B_data, dim, dim);
		printf("\nA'A\n");
		covariance_fast(B_data, dim, dim);
		printf("\n");
		hshld(B_data, dim, d, e);
		printf("\ndiags: \n");
		print_matrix(B_data, dim, dim);
	} else
	{
		printf("did not get array!\n");
		return 1;
	}
}
