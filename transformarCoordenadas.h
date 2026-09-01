

#include <iostream>
#include <fstream>
#include <Eigen\Eigen>
#include <math.h>
#include "Dynamixel.h"

//#define Torso_x 0.1
//#define Torso_y 0.2
//#define Torso_z 0.1

//#define Lower_Arm_Length 0.043
//#define Hand_Radius 0.0275
//#define Upper_Arm_Length 0.09

typedef unsigned char byte;

using namespace Eigen;
using namespace std;



class transformarCoordenadas {

	private :
		
		static Matrix3f A;
		static Vector3f manoI, hombroI, codoI, caderaI, manoD, hombroD,codoD,caderaD,v1,v2,v3,vtemp,c__,e1, e2, e3,solmi,solmd,solci,solcd,solhd,solhi,puntomanoder,puntomanoizq;		
		static int rspa_angle,rsa_angle,re_angle,rR_spa_angle,rR_sa_angle,rR_e_angle;
		float a,b,c,d,e,f,g,h,i;
        
	


	

			
		void setManoIKinect(Vector3f manoIzquierda);
		void setCodoIKinect(Vector3f codoIzquierda);
		void setHombroIKinect(Vector3f hombroIzquierda);
		void setCaderaIKinect(Vector3f caderaIzquierda);

		void setManoDKinect(Vector3f manoDerecha);
		void setCodoDKinect(Vector3f codoDerecha);
		void setHombroDKinect(Vector3f hombroDerecha);
		void setCaderaDKinect(Vector3f caderaDerecha);

		void setIzquierdo(int a, int b,int c);
		void setDerecho(int aa, int bb,int cc);

		
		
		float dist(Vector3f x, Vector3f y);

		void generarVectores();
		void normalizar();
		void generarMatriz();
		float cosAngle(Vector3f e, Vector3f V);
		void resolverPosiciones();	
		void escalar();
		void calcularCinematica(Vector3f mIzquierda,SerialPort *serialPort);
		void calcularCinematicaDerecha(Vector3f mDerecha,SerialPort *serialPort);



public  :


		transformarCoordenadas();
		~transformarCoordenadas();		
		
		void getPosicionesKinect (Vector3f manoIzq,Vector3f codoIzq, Vector3f hombroIzq, Vector3f caderaIzq, Vector3f manoDer, Vector3f codoDer, Vector3f hombroDer, Vector3f caderaDer);
		
		void transformar(SerialPort *serialPort);
		void tossMode (SerialPort *serialPort);
		
		Vector3f auxiliar;
			struct esqueleto {
			Vector3f manoIz;
			Vector3f hombroIz;
			Vector3f codoIz;
			Vector3f caderaIz;
			Vector3f manoDe;
			Vector3f hombroDe;
			Vector3f codoDe;
			Vector3f caderaDe;
			Vector3f solmiz;
			Vector3f solmde;
			Vector3f solciz;
			Vector3f solcde;
			Vector3f solhde;
			Vector3f solhiz;
			Vector3f puntomanodera;
			Vector3f puntomanoizqa;
			} esquelet;

			


		Vector3f getManoIKinect();
		Vector3f getCodoIKinect();
		Vector3f getHombroIKinect();
		Vector3f getCaderaIKinect();

		Vector3f getManoDKinect();
		Vector3f getCodoDKinect();
		Vector3f getHombroDKinect();
		Vector3f getCaderaDKinect();

		Vector3f getpuntoManoDer();
		Vector3f getpuntoManoIzq();

		


		void toHexHLConversion(short pos, byte *hexH, byte *hexL);
		short fromHexHLConversion(byte hexH, byte hexL);
		byte checkSumatory(byte  data[], int length);

};