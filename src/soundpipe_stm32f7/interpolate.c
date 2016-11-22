/*

Interpolate functions for octapal

*/



float elephant (float x, float y[])
{
	float z = x - 1/2.0;
	float even1 = y[1]+y[0], odd1 = y[1]-y[0];
	float even2 = y[2]+y[-1], odd2 = y[2]-y[-1];
	float c0 = even1*0.45868970870461956 + even2*0.04131401926395584;
	float c1 = odd1*0.48068024766578432 + odd2*0.17577925564495955;
	float c2 = even1*-0.246185007019907091 + even2*0.24614027139700284;
	float c3 = odd1*-0.36030925263849456 + odd2*0.10174985775982505;
	return ((c3*z+c2)*z+c1)*z+c0;
}

void interp5( float a[], int n, float b[], int m )
{
    float step = (float)( (float)n - 1 ) / ((float)m - 1);
    for( int j = 0; j < m; j ++ ){
        b[j] = elephant( j*step, a);
    }
}

float interp1( float x, float a[], int n )
{
    if( x <= 0 )  return a[0];
    if( x >= n - 1 )  return a[n-1];
    int j = (int)x;
    return a[j] + (x - j) * (a[j+1] - a[j]);
}

// float lerp(float a, float b, float f)
// {
//     return (a * (1.0f - f)) + (b * f);
// }


    // linear interpolate array a[] -> array b[]
void inter1parray( float a[], int n, float b[], int m )
{
    float step = (float)( (float)n - 1 ) / ((float)m - 1);
    for( int j = 0; j < m; j ++ ){
        b[j] = interp1( j*step, a, n );
    }
}



//..............................................................................
    // parabola through 3 points, -1 < x < 1

float parabola( float x, float f_1, float f0, float f1 )
{
    if( x <= -1 )  return f_1; 
    if( x >= 1 )  return f1; 
    float l = f0 - x * (f_1 - f0);
    float r = f0 + x * (f1 - f0);
    return (l + r + x * (r - l)) / 2;
}

    // quadratic interpolate x in an array
float interp2( float x, float a[], int n )
{
    if( x <= .5  ||  x >= n - 1.5 )
        return interp1( x, a, n );
    int j = (int)( x + .5 );
    float t = 2 * (x - j);  // -1 .. 1
    return parabola( t, (a[j-1] + a[j]) / 2, a[j], (a[j] + a[j+1]) / 2 );
}

    // quadratic interpolate array a[] -> array b[]
void interp2array( float a[], int n, float b[], int m )
{
    float step = (float)( n - 1 ) / (m - 1);
    for( int j = 0; j < m; j ++ ){
        b[j] = interp2( j*step, a, n );
    }
}