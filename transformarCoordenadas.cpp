
#include "stdafx.h"
#include "transformarCoordenadas.h"
#include <iostream>
#include <sstream>
#include <string>

using namespace std;
//#include "Dynamixel.h"

#define PI 3.14159265

//#define Torso_x .1
//#define Torso_y .2
//#define Torso_z .1

//#define Lower_Arm_Length .43
//#define Hand_Radius .0275
//#define Upper_Arm_Length .09



Matrix3f transformarCoordenadas::A;
Vector3f transformarCoordenadas::manoI;
Vector3f transformarCoordenadas::hombroI;
Vector3f transformarCoordenadas::codoI;
Vector3f transformarCoordenadas::caderaI;
Vector3f transformarCoordenadas::manoD;
Vector3f transformarCoordenadas::hombroD;
Vector3f transformarCoordenadas::codoD;
Vector3f transformarCoordenadas::caderaD;
Vector3f transformarCoordenadas::v1;
Vector3f transformarCoordenadas::v2;
Vector3f transformarCoordenadas::v3;
Vector3f transformarCoordenadas::vtemp;
Vector3f transformarCoordenadas::c__;
Vector3f transformarCoordenadas::e1;
Vector3f transformarCoordenadas::e2;
Vector3f transformarCoordenadas::e3;
Vector3f transformarCoordenadas::solmi;
Vector3f transformarCoordenadas::solmd;
Vector3f transformarCoordenadas::solci;
Vector3f transformarCoordenadas::solcd;
Vector3f transformarCoordenadas::solhd;
Vector3f transformarCoordenadas::solhi;
Vector3f transformarCoordenadas::puntomanoder;
Vector3f transformarCoordenadas::puntomanoizq;




int transformarCoordenadas::rspa_angle,transformarCoordenadas::rsa_angle,transformarCoordenadas::re_angle,transformarCoordenadas::rR_spa_angle,transformarCoordenadas::rR_sa_angle,transformarCoordenadas::rR_e_angle;

Vector3f aResolver, iNicio, v_hi_a_ci , v_ci_a_mi ,  v_hd_a_cd ,v_cd_a_md, ajusteManoDerecha, ajusteManoIzquierda,rvectorAxis,rvectorPos,rvectorDAxis,rvectorDPos,R_Shoulder_Joint,L_Shoulder_Joint;


float a,b,c,d,e,f,g,h,i,total_lower_arm_length;
float dot;

const float d_bioloid_brazo = 0.09;
const float d_bioloid_antebrazo = 0.043;
float d_brazo,d_antebrazo,d_todo_kinect,d_todo_bioloid; 
float CC,DD,C,D,Lower_Arm_Length,Hand_Radius,Upper_Arm_Length,shoulder_pivot_angle,elbow_angle,shoulder_angle,R_shoulder_pivot_angle,R_elbow_angle,R_shoulder_angle,Torso_x,Torso_y,Torso_z;
int spa_angle,sa_angle,e_angle,R_spa_angle,R_sa_angle,R_e_angle;


//double Torso_x,Torso_y,Torso_z,Lower_Arm_Length,Hand_Radius,Upper_Arm_Length;

	SerialPort serialPort;
	Dynamixel dynamixel;

	int waitTime=0;
	const int BufferSize=1024;
	byte buffer[BufferSize];
	byte bufferIn[BufferSize];
	

transformarCoordenadas::transformarCoordenadas()
{

	e1 << 1,0,0;
    e2 << 0,1,0;
	e3 << 0,0,1;
	iNicio << 0,0,0;
	ajusteManoDerecha << -0.05, 0.05, 0.025;
	ajusteManoIzquierda << 0.05, 0.05, 0.025;

	C = 0;
	D = 0;
	CC = 0 ;
	DD = 0;
	
	shoulder_pivot_angle = 0;
	elbow_angle = 0;
	shoulder_angle = 0;

	R_shoulder_pivot_angle = 0;
	R_elbow_angle = 0;
	R_shoulder_angle = 0;

	rvectorAxis << 0,0,0;
	rvectorPos << 0,0,0.2;

	rvectorDAxis << 0,0,0;
	rvectorDPos << 0,0,0.2;
	
	Lower_Arm_Length = .043;
	Hand_Radius = .275;
	Upper_Arm_Length = .09;
	
	spa_angle=0;
	sa_angle=0;
	e_angle=0;

	R_spa_angle=0;
	R_sa_angle=0;
	R_e_angle=0;

	Torso_x = 0.1;
	Torso_y = 0.2;
	Torso_z = 0.1;


	R_Shoulder_Joint <<  Torso_x/2, Torso_y/4, Torso_z/4; 
	L_Shoulder_Joint << -Torso_x/2, Torso_y/4, Torso_z/4; 

	
}
	
transformarCoordenadas::~transformarCoordenadas(){
}





void transformarCoordenadas::getPosicionesKinect(Vector3f manoIzq,Vector3f codoIzq, Vector3f hombroIzq, Vector3f caderaIzq, Vector3f manoDer, Vector3f codoDer, Vector3f hombroDer, Vector3f caderaDer){

	setManoIKinect(manoIzq);
	setCodoIKinect(codoIzq);
	setHombroIKinect(hombroIzq);
	setCaderaIKinect(caderaIzq);

	setManoDKinect(manoDer);
	setCodoDKinect(codoDer);
	setHombroDKinect(hombroDer);
	setCaderaDKinect(caderaDer);

	
	

}

void transformarCoordenadas::tossMode(SerialPort *serialPort){

	dynamixel.sentTossModeCommand(serialPort); 

}

void transformarCoordenadas::transformar(SerialPort *serialPort){
	
	generarVectores();
	normalizar();
	generarMatriz();
	resolverPosiciones();
	escalar();
	
	calcularCinematica(puntomanoizq,serialPort);
	calcularCinematicaDerecha(puntomanoder,serialPort);

}

void transformarCoordenadas::setManoIKinect(Vector3f mano){
	manoI = mano;
}

void transformarCoordenadas::setCodoIKinect(Vector3f codo){
	codoI = codo;
}
   
void transformarCoordenadas::setHombroIKinect(Vector3f hombro){
	hombroI = hombro;
}

void transformarCoordenadas::setCaderaIKinect(Vector3f cadera){
	caderaI = cadera;
}
void transformarCoordenadas::setManoDKinect(Vector3f mano){
	manoD= mano;
}
void transformarCoordenadas::setCodoDKinect(Vector3f codo){
	codoD = codo;
}
   
void transformarCoordenadas::setHombroDKinect(Vector3f hombro){
	hombroD = hombro;
}

void transformarCoordenadas::setCaderaDKinect(Vector3f cadera){
	caderaD = cadera;
}


Vector3f transformarCoordenadas::getManoIKinect(){

	//return manoI;
	return solmi;
}

Vector3f transformarCoordenadas::getCodoIKinect(){
	//return codoI;
	return solci;
}
   
Vector3f transformarCoordenadas::getHombroIKinect(){
	
	//return hombroI;
	return solhi;
}

Vector3f transformarCoordenadas::getCaderaIKinect(){
	return caderaI;
	
}
Vector3f transformarCoordenadas::getManoDKinect(){
	//return manoD;
	return solmd;
}
Vector3f transformarCoordenadas::getCodoDKinect(){
	//return codoD;
	return solcd;
}
   
Vector3f transformarCoordenadas::getHombroDKinect(){
	//return hombroD;
	return solhd;
}

Vector3f transformarCoordenadas::getCaderaDKinect(){
	return caderaD;
}
Vector3f transformarCoordenadas::getpuntoManoDer(){
	return puntomanoder;
}
Vector3f transformarCoordenadas::getpuntoManoIzq(){
	return puntomanoizq;
}

void transformarCoordenadas::generarVectores(){

	v1 = hombroI - hombroD;
	vtemp = hombroI - caderaI;
	//dot = (float (vtemp.dot(v1)) / float(v1.dot(v1)));
	//dot = ((vtemp.dot(v1)) / (v1.dot(v1)));
	//c__ = caderaI + dot *v1;
	c__ = caderaI + (float (vtemp.dot(v1)) / float(v1.dot(v1))) * v1;	
	v2 = hombroI - c__;
	
	v3 = v1.cross(v2);

}

void transformarCoordenadas::normalizar(){

	v1 = v1 / sqrt (v1.cwiseAbs2().sum());
	v2 = v2 / sqrt (v2.cwiseAbs2().sum());
	v3 = v3 / sqrt (v3.cwiseAbs2().sum());
}

float transformarCoordenadas::cosAngle(Vector3f e, Vector3f V){

	float resultado, dotA, x_modulus,y_modulus;
	dotA = e.dot(V);
	x_modulus = sqrt (e1.cwiseAbs2().sum());
	y_modulus = sqrt (V.cwiseAbs2().sum());

	return resultado = dotA / x_modulus/ y_modulus;

}


void transformarCoordenadas::generarMatriz(){
	
	a = cosAngle(e1,v1);
	b = cosAngle(e2,v1);
	c = cosAngle(e3,v1);
	d = cosAngle(e1,v2);
	e = cosAngle(e2,v2);
	f = cosAngle(e3,v2);
	g = cosAngle(e1,v3);
	h = cosAngle(e2,v3);
	i = cosAngle(e3,v3);

	A << a , d , g ,  b , e , h , c , f , i ;


}

void transformarCoordenadas::resolverPosiciones(){
	
	 aResolver = manoI - hombroI;	 
	 solmi = A.colPivHouseholderQr().solve(aResolver);
	 aResolver = manoD - hombroI;
     solmd = A.colPivHouseholderQr().solve(aResolver);
	 aResolver = codoI - hombroI;
     solci = A.colPivHouseholderQr().solve(aResolver);
	 aResolver = codoD - hombroI;
     solcd = A.colPivHouseholderQr().solve(aResolver);
	 aResolver = hombroD - hombroI;
     solhd = A.colPivHouseholderQr().solve(aResolver);

	// ofstream fs("prueba.txt",ios::app); 

	//fs << " SOL_Hombro Izquierdo " << solhi << endl;
	//fs << " SOL_Codo Izquierdo   " << solci << endl;
	//fs << " SOL_Mano Izquierdo   " << solmi << endl;
	//fs << " SOL_Hombro Derecho   " << solhd << endl;
	//fs << " SOL_Codo Derecho     " << solcd << endl;
	//fs << " SOL_Mano Derecho     " << solmd << endl;

	//fs.close();
}

float transformarCoordenadas::dist(Vector3f x, Vector3f y){

	Vector3f resultado;
	resultado = x - y;
	return sqrt (resultado.cwiseAbs2().sum());
}

void transformarCoordenadas::escalar(){

	/* d_brazo = dist (iNicio,solci);
     d_antebrazo = dist (solci,solmi);*/
	 d_todo_kinect = d_brazo + d_antebrazo;
	 d_todo_bioloid = d_bioloid_brazo + d_bioloid_antebrazo;


	 v_hi_a_ci = (solci / sqrt (solci.cwiseAbs2().sum())) * d_bioloid_brazo;
	 v_ci_a_mi  = (solmi - solci);
	 v_ci_a_mi  = (v_ci_a_mi  / sqrt (v_ci_a_mi.cwiseAbs2().sum())) * d_bioloid_antebrazo;
	 puntomanoizq = v_hi_a_ci + v_ci_a_mi + ajusteManoIzquierda;


	 v_hd_a_cd = (solcd - solhd);
	 v_hd_a_cd = (v_hd_a_cd  / sqrt (v_hd_a_cd.cwiseAbs2().sum())) *  d_bioloid_brazo;
	 v_cd_a_md = (solmd - solcd);
	 v_cd_a_md = (v_cd_a_md  /  sqrt (v_cd_a_md.cwiseAbs2().sum())) *  d_bioloid_antebrazo;
	 puntomanoder = v_hd_a_cd + v_cd_a_md + ajusteManoDerecha;

	
}

void transformarCoordenadas::setIzquierdo(int a , int b, int c){
	rspa_angle = a;
	rsa_angle = b;
	re_angle = c;
}
void transformarCoordenadas::setDerecho(int aa , int bb, int cc){
	rR_spa_angle = aa;
	rR_sa_angle = bb;
	rR_e_angle = cc;
}



void transformarCoordenadas::calcularCinematica(Vector3f mIzquierda,SerialPort *serialPort){

	//mano Izquierda
			 
	if ((mIzquierda(0) > Torso_x/2  || mIzquierda(0) < -Torso_x/2) || (mIzquierda(1) > Torso_y/2  || mIzquierda(0) < -Torso_y/2) || (mIzquierda(2) > Torso_z/2  || mIzquierda(2) < -Torso_z/2)){
	
		rvectorAxis = mIzquierda - R_Shoulder_Joint;
		rvectorPos = R_Shoulder_Joint;
		total_lower_arm_length = Lower_Arm_Length;
		//Mirar si el punto esta en el dominio. C es una variable temporal
        //igual a la diferencia enrte la extension maxima del brazo y el puntodeseado
		//  C = (mag(rvector.axis)**2 - Upper_Arm_Length**2 - total_lower_arm_length**2)/(2*Upper_Arm_Length*total_lower_arm_length)
        //if C**2 <= 1: //Computar cinematica inversa
		//C = ((sqrt (rvectorAxis.cwiseAbs2().sum()) * sqrt (rvectorAxis.cwiseAbs2().sum())) - (Upper_Arm_Length*Upper_Arm_Length) - (total_lower_arm_length * total_lower_arm_length)) / (2 * Upper_Arm_Length * total_lower_arm_length);
		


		C = ((rvectorAxis.norm()*rvectorAxis.norm()) - (Upper_Arm_Length*Upper_Arm_Length) - (total_lower_arm_length * total_lower_arm_length)) / (2 * Upper_Arm_Length * total_lower_arm_length);		
		
		
		if  ((C*C) <= 1 ){
		
		
			/*	fs << " SOL_Hombro Izquierdo " << solhi << endl;
				fs << " hombro Izquierda     " << hombroI << endl;
				fs << " SOL_Codo Izquierdo   " << solci << endl;
				fs << " codo Izquierda       " << codoI << endl;
				fs << " SOL_Mano Izquierdo   " << solmi << endl;
				fs << " mano Izquierda       " << manoI << endl;
				fs << " SOL_Hombro Derecho   " << solhd << endl;
				fs << " hombro Derecho       " << hombroD << endl;
				fs << " SOL_Codo Derecho     " << solcd << endl;
				fs << " codo Derecho         " << codoD << endl;
				fs << " SOL_Mano Derecho     " << solmd << endl;
				fs << " mano derecha         " << manoD << endl;
			fs.close();*/
			
			D = sqrt (1-(C*C));
			elbow_angle = atan2(D,C);
			//fs << "  rvectorAxis(0)    " << rvectorAxis(0) << endl;
			if (rvectorAxis(0)  > 0){
				//  shoulder_angle = -(atan2(rvector.axis.x,sqrt(rvector.axis.z**2+rvector.axis.y**2))\+atan2(total_lower_arm_length*sin(elbow_angle),(Upper_Arm_Length+total_lower_arm_length*cos(elbow_angle))))+pi/2
		 		shoulder_angle = -(atan2(rvectorAxis(0),sqrt((rvectorAxis(2)*rvectorAxis(2))+(rvectorAxis(1)*rvectorAxis(1))))  + atan2(total_lower_arm_length*sin(elbow_angle),(Upper_Arm_Length+total_lower_arm_length*cos(elbow_angle)))) + (PI/2);				
			}else{
				//shoulder_angle = (atan2(-rvector.axis.x,sqrt(rvector.axis.z**2+rvector.axis.y**2))\-atan2(total_lower_arm_length*sin(elbow_angle),(Upper_Arm_Length+total_lower_arm_length*cos(elbow_angle))))+pi/2
				shoulder_angle = (atan2(-rvectorAxis(0),sqrt((rvectorAxis(2)*rvectorAxis(2))+(rvectorAxis(1)*rvectorAxis(1)))) - atan2(total_lower_arm_length*sin(elbow_angle),(Upper_Arm_Length+total_lower_arm_length*cos(elbow_angle)))) + (PI/2);				
			}
				//fs << "  shoulder_angle    " << shoulder_angle << endl;
				/*Z se vuelve Y y X se vuelve Z asi que Y se vuelve X
				shoulder_pivot_angle = -atan2(rvector.axis.y,rvector.axis.z)
                shoulder_pivot_angle *= 180 / np.pi #pasamos a grados
                shoulder_angle *= 180 / np.pi
                elbow_angle *= 180 / np.pi*/
				shoulder_pivot_angle = -atan2(rvectorAxis(1),rvectorAxis(2));
                shoulder_pivot_angle *= 180 / PI;// np.pi #pasamos a grados
                shoulder_angle *= 180 / PI;
                elbow_angle *= 180 / PI;
				
				//fs << "  shoulder_pivot_angle    " << shoulder_pivot_angle << endl;
				//fs << "  shoulder_angle          " << shoulder_angle << endl;
				//fs << "  elbow_angle             " << elbow_angle << endl;

				/*Formamos los mensajes para el bioloid, los +146 son debidos a laposicion inicial de los servos
                #para que sean el 0 que nosotros esperamos tenemos que sumarle esa cantidad
                spa_angle = int(3.5*( (shoulder_pivot_angle + 146) % 360) )
                comando_spa = "%d %d\n" % (2, int(spa_angle)) # sintaxis comando bioloid"%d %d\n" num_motor cantidad
                sa_angle = int(3.5*( (shoulder_angle + 146) % 360))
                comando_sa = "%d %d\n" % (4, int(sa_angle))
                e_angle = int(3.5*( (elbow_angle + 146) % 360 ) )
                comando_e = "%d %d\n" % (6, int(e_angle))*/
				/*spa_angle = int(3.5*(((int (shoulder_pivot_angle)  + 146) * 100)/ 360));
				sa_angle =  int(3.5*(((int (shoulder_angle)        + 146) * 100)/ 360));
				e_angle =   int(3.5*(((int (elbow_angle)           + 146) * 100)/ 360));*/

				spa_angle = int(3.5*((int (shoulder_pivot_angle)  + 146)  % 360));
				sa_angle =  int(3.5*((int (shoulder_angle)        + 146)  % 360));
				e_angle =   int(3.5*((int (elbow_angle)           + 146)  % 360));

				/*fs << "  Bioloid shoulder_pivot_angle    " << spa_angle << endl;
				fs << "  Bioloid shoulder_angle          " << sa_angle << endl;
				fs << "  Bioloid elbow_angle             " << e_angle << endl;*/
				
					/*ofstream fs("prueba.txt",ios::app);
	                fs << hex << spa_angle << endl;
					fs << hex << sa_angle << endl;
					fs << hex << e_angle << endl;
					fs.close();*/

   // 		if (serialPort.connect(L"COM3")==0) {	
			//		//dynamixel.sentTossModeCommand(&serialPort);
			//		/*dynamixel.setPosition(&serialPort, 0, spa_angle);
			//		dynamixel.setPosition(&serialPort, 1, sa_angle);
			//		dynamixel.setPosition(&serialPort, 2, e_angle);*/
			//	//dynamixel.setPositionSyn(&serialPort,spa_angle,sa_angle,e_angle,R_spa_angle,R_sa_angle,R_e_angle);
				dynamixel.setPositionSyn(serialPort,spa_angle,sa_angle,e_angle);
			//	   
			//}else{
			//
			//}
			//serialPort.disconnect();
			
			/*stringstream oss,oss1,ss,ss1,o,o1;	
			int  centena = 0, decena = 0, millar = 0, unidad =0;
			byte spa1,spa2;
			byte sa1,sa2;
			byte e1,e2;
			long int n3;*/

			//byte b = (byte)integerValue


				//if (spa_angle >= 1000){
				//	unidad=spa_angle%10;
				//	decena=(spa_angle/10)%10;
				//	centena=(spa_angle/100)%10;
				//	millar=(spa_angle/1000)%10;	
				//	
				//	oss << millar << centena;
				//	n3 = strtol(oss.str().c_str(), NULL, 10);
				//	spa2 = (byte)n3;
				//	oss1 << decena << unidad;
				//	n3 = strtol(oss1.str().c_str(), NULL, 10);
				//	spa1 = (byte)n3;

				//}else if (spa_angle < 1000 & spa_angle >= 100){
				//	unidad=spa_angle%10;
				//	decena=(spa_angle/10)%10;
				//	centena=(spa_angle/100)%10;
				//	spa2 = (byte)centena;
				//	oss << decena << unidad;
				//	n3 = strtol(oss.str().c_str(), NULL, 10);
				//	spa1 = (byte)n3;
				//}else if (spa_angle < 100 & spa_angle >10){
				//	unidad=spa_angle%10;
				//	decena=(spa_angle/10)%10;
				//	spa2 = (byte)0;
				//	oss << decena << unidad;
				//	n3 = strtol(oss.str().c_str(), NULL, 10);
				//	spa1 = (byte)n3;
				//}else{
				//	unidad=spa_angle%10;
				//	spa2 = (byte)0;
				//	spa1 = (byte)unidad;
				//}
				//

			

				//if (sa_angle >= 1000){
				//	unidad=sa_angle%10;
				//	decena=(sa_angle/10)%10;
				//	centena=(sa_angle/100)%10;
				//	millar=(sa_angle/1000)%10;	
				//	
				//	ss << millar << centena;
				//	n3 = strtol(oss.str().c_str(), NULL, 10);
				//	sa2 = (byte)n3;
				//	ss1 << decena << unidad;
				//	n3 = strtol(ss1.str().c_str(), NULL, 10);
				//	sa1 = (byte)n3;

				//}else if (sa_angle < 1000 & sa_angle >= 100){
				//	unidad=sa_angle%10;
				//	decena=(sa_angle/10)%10;
				//	centena=(sa_angle/100)%10;
				//	sa2 = (byte)centena;
				//	ss << decena << unidad;
				//	n3 = strtol(ss.str().c_str(), NULL, 10);
				//	sa1 = (byte)n3;
				//}else if (sa_angle < 100 & sa_angle >10){
				//	unidad=sa_angle%10;
				//	decena=(sa_angle/10)%10;
				//	sa2 = (byte)0;
				//	ss << decena << unidad;
				//	n3 = strtol(ss.str().c_str(), NULL, 10);
				//	sa1 = (byte)n3;
				//}else{
				//	unidad=sa_angle%10;
				//	sa2 = (byte)0;
				//	sa1 = (byte)unidad;
				//}


				//if (e_angle >= 1000){
				//	unidad=e_angle%10;
				//	decena=(e_angle/10)%10;
				//	centena=(e_angle/100)%10;
				//	millar=(e_angle/1000)%10;	
				//	o << millar << centena;
				//	n3 = strtol(o.str().c_str(), NULL, 10);
				//	e2 = (byte)n3;
				//	o1 << decena << unidad;
				//	n3 = strtol(o1.str().c_str(), NULL, 10);
				//	e1 = (byte)n3;

				//}else if (e_angle < 1000 & e_angle >= 100){
				//	unidad=e_angle%10;
				//	decena=(e_angle/10)%10;
				//	centena=(e_angle/100)%10;
				//	e2 = (byte)centena;
				//	o << decena << unidad;
				//	n3 = strtol(o.str().c_str(), NULL, 10);
				//	e1 = (byte)n3;
				//}else if (e_angle < 100 & e_angle >10){
				//	unidad=e_angle%10;
				//	decena=(e_angle/10)%10;
				//	e2 = (byte)0;
				//	o << decena << unidad;
				//	n3 = strtol(o.str().c_str(), NULL, 10);
				//	e1 = (byte)n3;
				//}else{
				//	unidad=e_angle%10;
				//	e2 = (byte)0;
				//	e1 = (byte)unidad;
				//}
				//	ofstream fs("prueba.txt",ios::app);
	   //             fs << "  spa_angle    " << spa2 << endl;
				//	fs << "  spa_angle    " << spa1 << endl;
				//	fs << "  sa_angle   " << sa1 << endl;
				//	fs << "  sa_angle    " << sa2 << endl;
				//	fs << "  e_angle    " << e1<< endl;
				//	fs << "  e_angle    " << e2 << endl;
				//	fs.close();


					//fs.close();
	}
	
	//mano Derecha

	//if ((mDerecha(0) > Torso_x/2  || mDerecha(0) < -Torso_x/2) || (mDerecha(1) > Torso_y/2  || mDerecha(0) < -Torso_y/2) || (mDerecha(2) > Torso_z/2  || mDerecha(2) < -Torso_z/2)){

		
		}
}
void transformarCoordenadas::calcularCinematicaDerecha(Vector3f mDerecha,SerialPort *serialPort){
	
	if ((mDerecha(0) > Torso_x/2  || mDerecha(0) < -Torso_x/2) || (mDerecha(1) > Torso_y/2  || mDerecha(0) < -Torso_y/2) || (mDerecha(2) > Torso_z/2  || mDerecha(2) < -Torso_z/2)){
	
	
	    rvectorDAxis = mDerecha - L_Shoulder_Joint;
		rvectorDPos = L_Shoulder_Joint;
		total_lower_arm_length = Lower_Arm_Length;
	
		
		CC = ((rvectorDAxis.norm()*rvectorDAxis.norm()) - (Upper_Arm_Length*Upper_Arm_Length) - (total_lower_arm_length * total_lower_arm_length)) / (2 * Upper_Arm_Length * total_lower_arm_length);		
		
	
		if ((CC*CC) <= 1){
			
			
	
			DD = sqrt (1-(CC*CC));
		
			R_elbow_angle = atan2(DD,CC);

			if (rvectorDAxis(0)  > 0){
				// shoulder_angle = (atan2(rvector.axis.x,sqrt(rvector.axis.z**2+rvector.axis.y**2))\135. -atan2(total_lower_arm_length*sin(elbow_angle),(Upper_Arm_Length+total_lower_arm_length*cos(elbow_angle))))+pi/2
		 		R_shoulder_angle = (atan2(rvectorDAxis(0),sqrt((rvectorDAxis(2)*rvectorDAxis(2))+(rvectorDAxis(1)*rvectorDAxis(1))))  - atan2(total_lower_arm_length*sin(R_elbow_angle),(Upper_Arm_Length+total_lower_arm_length*cos(R_elbow_angle)))) + (PI/2);				
			}else{
				//shoulder_angle = -(atan2(-rvector.axis.x,sqrt(rvector.axis.z**2+rvector.axis.y**2))\138. +atan2(total_lower_arm_length*sin(elbow_angle),(Upper_Arm_Length+total_lower_arm_length*cos(elbow_angle))))+pi/2
				R_shoulder_angle = - (atan2(-rvectorDAxis(0),sqrt((rvectorDAxis(2)*rvectorDAxis(2))+(rvectorDAxis(1)*rvectorDAxis(1)))) + atan2(total_lower_arm_length*sin(R_elbow_angle),(Upper_Arm_Length+total_lower_arm_length*cos(R_elbow_angle)))) + (PI/2);				
			}

				R_shoulder_pivot_angle = -atan2(rvectorDAxis(1),rvectorDAxis(2));
                R_shoulder_pivot_angle *= 180 / PI;// np.pi #pasamos a grados
                R_shoulder_angle *= 180 / PI;
                R_elbow_angle *= 180 / PI;

	/*			fs << "  r_shoulder_pivot    " << R_shoulder_pivot_angle << endl;
				fs << "  R_shoulder_angle    " << R_shoulder_angle << endl;
				fs << "  R_elbow_angle    " << R_elbow_angle << endl;*/

				R_spa_angle = int(3.5*((int (-R_shoulder_pivot_angle)  + 146)  % 360));
				R_sa_angle =  int(3.5*((int (-R_shoulder_angle)        + 146)  % 360));
				R_e_angle =   int(3.5*((int (-R_elbow_angle)           + 146)  % 360));


				/*fs << "  r_shoulder_pivot    " << R_spa_angle << endl;
				fs << "  R_shoulder_angle    " << R_sa_angle << endl;
				fs << "  R_elbow_angle    " << R_e_angle << endl;*/
				
			//	if (serialPort.connect(L"COM3")==0) {	
			//		//dynamixel.sentTossModeCommand(&serialPort);
			//		/*dynamixel.setPosition(&serialPort, 0, spa_angle);
			//		dynamixel.setPosition(&serialPort, 1, sa_angle);
			//		dynamixel.setPosition(&serialPort, 2, e_angle);*/
			//	//dynamixel.setPositionSyn(&serialPort,spa_angle,sa_angle,e_angle,R_spa_angle,R_sa_angle,R_e_angle);
				//dynamixel.setPositionSynDerecha(&serialPort,R_spa_angle,R_sa_angle,R_e_angle);
			//	dynamixel.setPositionSynDerecha(serialPort,R_spa_angle,R_sa_angle,R_e_angle);
			//	   
			//}else{
			//
			//}
			//serialPort.disconnect();

		}
	}

}

