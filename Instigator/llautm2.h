//Written By: Owen Allen
// Updated By: Mike Franks (9 MAY 10)
//  Changed many values to const instead of performing calculations each time
//  Fixed problem of not accepting negitive long. values

#include<math.h>

//LLA_UTM2
// Example Main Code for conversion: 

//       utmCoordinates utmCoordinatesOut;	                    (Global variable)
//       utmCoordinatesOut = *LLA_UTM2(fLatitude, fLongitude);  (Code in Main())	

//Takes in lat, lon (degrees) and returns UTM Northing, Easting, Zone
//(meters)
//Note on use: for lat,lon: N,E are positive; S,W are negative
//Refrence: Map Projections - A Working Model, 
//US Geolocial Survey Professional Paper 1395
//See page 61 onward
//See page 261 for examples
#define pi 3.14159265
typedef struct { double xCoordinate; double yCoordinate; }utmCoordinates;  


utmCoordinates * LLA_UTM2(double lat, double lon)
{
//Geodetic refrence system constants
//GRS80
//m_a_c = 6378137.0; //equitorial radius or semimajor axis of the ellipsode //reference
//m_b_c = 6356752.31414;
//WGS84
//m_a_c = 6378137.0; //equitorial radius or semimajor axis of the ellipsode //reference
//m_b_c = 6356752.31425;
 
//Reference Ellipsoid Properties
double a = 6378137.0;//6378137.0; //equitorial radius or semimajor axis of the ellipsode reference
//double b = 6356752.31425;
double e2 = .00669437998863; //(1-pow(b,2)/pow(a,2));// e^2 = (1-b^2/a^2) the eccentricity of the ellipsod
 							  //^ WHY WAS THIS BEING CALCULATED EACH TIME!?

//Convert lat, lon from degrees to radians
double phi = pi*(lat/180); //lattitude		// all but phi and lam can be floats possibly?
double lam = pi*(lon/180); //longitude
double lon0 = 0.0;
double lam0 = 0.0;
double k0 = 0.0;
double x0 = 0.0; 
double e2p = 0.0;
double N = 0.0;
double T = 0.0;
double C = 0.0;
double A = 0.0;
double M1 = 0.0;
double M2 = 0.0;
double M3 = 0.0;
double M4 = 0.0;
double M = 0.0;
double M0 = 0.0;
double x1 = 0.0;
double x2 = 0.0;
double x = 0.0;
double y1 = 0.0;
double y2 = 0.0;
double y3 = 0.0;
double y = 0.0;
static utmCoordinates ret;
//Find zone
double zone = floor(lon/6) + 31;
if (zone == 61)
    zone = 60;


//Find lon0
lon0 = 6*(zone-30)-3;

lam0 = pi*lon0/180;

// zone = 1;
// if lon >= 0 //Longitude is in the Eastern hemisphere
//     zone = 31 + floor(lon/6)
// else //Longitude is in the Western hemisphere
//     zone = 30 + ceil(lon/6)
// end
// 
// lon0 = -180 + (zone-1)*6+3
// lam0 = pi*lon0/180

k0 = 0.9996; //scale on central meridian
x0 = 500000; //False Easting

//Equations 8-12 through 8-15, pp 1395
e2p = .0067394967407447;  //e2 / (1 - e2);
N = a/sqrt(1-e2*pow(sin(phi),2));
T = pow(tan(phi),2);
C = e2p*pow(cos(phi),2);
A = (lam - lam0)*cos(phi);

//M is true distance along central meridian from the Equator to phi.
//Equ 3-21, pp 1395
M1 = .99832429845317 * phi;   // (1-e2/4 -3*pow(e2,2)/64 - 5*pow(e2,3)/256)*phi;
M2 = .0025146070599501 * sin(2*phi); //(3*e2/8 + 3*pow(e2,2)/32 + 45*pow(e2,3)/1024)*sin(2*phi);
M3 = 2.6390465931431e-6 * sin(4*phi); //(15*pow(e2,2)/256 + 45*pow(e2,3)/1024)*sin(4*phi);
M4 = 3.4180460842809e-9 * sin(6*phi);//(35*pow(e2,3)/3072)*sin(6*phi);
M = a*( M1 - M2 + M3 - M4 );
M0 = 0;

//equation 8-9, pp 1395 Note splitting of equation for easier handling
x1 = A + (1-T+C)*pow(A,3)/6;
x2 = (5-18*T + pow(T,2)+72*C-.38708908109632)*pow(A,5)/120; //(5-18*T + pow(T,2)+72*C-58*e2p)*pow(A,5)/120;
x = k0*N*(x1 + x2) + x0;//False easting may be added here

//equation 8-9, pp 1395 Note splitting of equation for easier handling
y1 = M - M0;
y2 = pow(A,2)/2 + (5-T+9*C+4*pow(C,2))*pow(A,4)/24;
y3 = (61-58*T+pow(T,2)+600*C-2.2024033924446)*pow(A,6)/720; //(61-58*T+pow(T,2)+600*C-330*e2p)*pow(A,6)/720;
y = k0*(y1 + N*tan(phi)*(y2+y3));
ret.xCoordinate = x;
ret.yCoordinate = y;
return &ret;
}
