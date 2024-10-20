#include <fstream>
#include <iostream>
#include <windows.h> //Permette l'uso della funzione sleep
#include <NIDAQmx.h> //Permette l'uso della libreria NIDAQmx
#include <math.h>
#include <conio.h> //Permette di uscire da un ciclo infinito premendo un tasto qualunque da tastiera
#include <stdio.h>
#include <string>

#define ErroreOnOff -1 // Numeri Random per definire i messaggi di errore
#define ErrorePid -2
#define NoErrors 0
#define TemperaturaMax 120
#define Nmis 1000000
#define FreqC 100 // Frequenza di campionamento
using namespace std;
double conv_temperatura(double mV);
int OnOff(int controllo, float Tsetpoint, int com);
double conv_temperatura(double mV) //Funzione che converte la tensione in mV in una temperatura in gradi Celsius
    {
        double c1, c2, c3, c4;
        double Temp;

        c1= -0.125079;
        c2= 1.04063;
        c3= 1.69477E-03;
        c4= 9.57745E-06;
        
        Temp= c1 + c2*mV + c3*mV*mV + c4*mV*mV*mV;
        return Temp;

    }
int OnOff_(int controllo, float Tsetpoint){
	// Programma per la lettura dei dati con stampa su file dell algoritmo OnOff
	float64 fValue; //Dichiaro variabile del valore da registrare
	int t=0;
	
	TaskHandle lettura, alimentazione;
	DAQmxCreateTask("", &lettura);
	DAQmxCreateTask("", &alimentazione);
	DAQmxCreateAOVoltageChan(alimentazione,"Dev1/ao0","",-10.0,10.0,DAQmx_Val_Volts,NULL);

	DAQmxCreateAIVoltageChan(lettura,"Dev1/ai0","",DAQmx_Val_Cfg_Default,0.0,100e-3,DAQmx_Val_Volts,"");
    DAQmxStartTask(alimentazione);
	DAQmxStartTask(lettura);
	ofstream fileout;
	char  name[80];
		
	cout << "dare nome file"<<endl;
	cin>> name;
	ofstream outfile (name);
	

	cout<<"Tensione \t Temperatura "<<endl;
	outfile<< "Tempo" <<"\t"<<"Temperatura_ONOFF"<<endl;
do
	{

		DAQmxReadAnalogScalarF64(lettura,10.0,&fValue,0);

		double temp = conv_temperatura(fValue*1000); //Conversione segnale mV a temperatura in gradi Celsius

		if (temp < Tsetpoint){
			DAQmxWriteAnalogScalarF64(alimentazione,1,10.0,4.0,NULL);
			cout<<"ON\n "<<endl;
			outfile<< 0.1*t <<"\t" <<4.0<<"\t"<<temp<<endl;
			
		}
		else{
			DAQmxWriteAnalogScalarF64(alimentazione,1,10.0,0.0,NULL);
			cout<<"OFF\n  "<<endl;
			outfile<< 0.1*t <<"\t" <<0.0<<"\t"<<temp<<endl;
			
		}
		
		cout<< fValue<<"\t"<<temp<<endl;
		Sleep(100);
		t++;

	} while (t<controllo );

	DAQmxWriteAnalogScalarF64(alimentazione,1,10.0,0.0,NULL);
	outfile.close();
	DAQmxStopTask(lettura);
	DAQmxStopTask(alimentazione);
	DAQmxClearTask(lettura);
	DAQmxClearTask(alimentazione);

	return NoErrors;
}

int PID_(double kp, double ki,double kd, double Tsetpoint){
	double err=0;
	float64 fValue; //Dichiaro variabile del valore da registrare 

	char  name[80];
		
	cout << "dare nome file"<<endl;
	cin>> name;
	ofstream outfile (name);

	TaskHandle lettura, alimentazione;
	DAQmxCreateTask("", &lettura);
	DAQmxCreateTask("", &alimentazione);
	DAQmxCreateAOVoltageChan(alimentazione,"Dev1/ao0","",0.0,10.0,DAQmx_Val_Volts,NULL);

	DAQmxCreateAIVoltageChan(lettura,"Dev1/ai0","",DAQmx_Val_Cfg_Default,0.0,100e-3,DAQmx_Val_Volts,"");
    DAQmxStartTask(alimentazione);
	DAQmxStartTask(lettura);
	
	double ei=0;//Errore integrale inizializzato
	double vetterr[10]={0};//Errore differenziale inizializzato 
// Acquisizione misure	
	int i=0;
	outfile << "Tempo (s)" <<"\t" << "Tensione input (V)" << "\t" << "Errore Proporzionale" <<"\t" << "Errore Integrale"<< "\t" << "Errore Derivata" <<"\t" << "Temperatura" <<"\t" <<endl;
	while( !(_kbhit()||i>Nmis-1)){
			
		DAQmxReadAnalogScalarF64(lettura,10.0,&fValue,0);
		double temp = conv_temperatura(fValue*1000);//Conversione segnale mV a temperatura in gradi Celsius
		// se la temperatura supera la Temperatura Massima blocca tutto
		if(temp>TemperaturaMax){
		DAQmxWriteAnalogScalarF64(alimentazione,1,10.0,0.0,NULL);
		outfile.close();
		DAQmxStopTask(lettura);
		DAQmxStopTask(alimentazione);
		DAQmxClearTask(lettura);
		DAQmxClearTask(alimentazione);
			return ErrorePid;
		}
		err=Tsetpoint-temp;//Funzione errore
		double ep=kp*err;// Errore proporzionale
	
		
		vetterr[i%10]=err;
		if(err<5)
			{
ei += ki*err*(0.001*FreqC);//Errore integrale
		}	else{ 
			ei=0;
	}

	double ed=kd*(err-vetterr[(i+1)%10])/(10*0.001*FreqC);//Errore differenziale
 double pot=ep+ei+ed;
		pot=max(0,pot);//Contributo totale
		outfile << i*0.1 <<"\t" << pot << "\t" << ep <<"\t" << ei << "\t" << ed <<"\t" << temp <<"\t" <<endl;
		cout<< i*0.1 <<"\t"<<fValue<<"\t"<<temp<<"\t"<<pot<<endl;

		DAQmxWriteAnalogScalarF64(alimentazione,1,10.0,min(pot,10.0),NULL);
		i++;
		Sleep(100);
	}
	DAQmxWriteAnalogScalarF64(alimentazione,1,10.0,0.0,NULL);
	outfile.close();
    DAQmxStopTask(lettura);
	DAQmxStopTask(alimentazione);
	DAQmxClearTask(lettura);
	DAQmxClearTask(alimentazione);

	return NoErrors;

}
int main (void)
{
	double temperatura=0.0;
	int opzione=0;
	float64 fValue=0.0;
do
{
	float Tsetpoint;
	
	//Apertura menu
	cout<< "Menu:"<< endl;
	cout<< "1-OnOff"<< endl;
	cout<< "2-PID"<< endl;
	cout<<"3-Controllo Temperatura"<< endl;
	cout<< "4-esc"<< endl;
	cin>>opzione;

	int status;

	switch (opzione)
	{
		case 1:
						// Imposto la temperatura di setpoint
			cout<< "Fornire temperatura di controllo (T massima 120)"<< endl;
			cin>>Tsetpoint;
			
		        status = OnOff_(Nmis, Tsetpoint);
			if(status== ErroreOnOff)
			cout<<"Errore: Superata Temperatura Massima "<<endl;
		
			break;
		break;
		case 2:
					// Imposto la temperatura di setpoint
			do{
		cout<< "Fornire temperatura di controllo (T massima 120)"<< endl;
		cin>>Tsetpoint;
			}while(Tsetpoint>TemperaturaMax);
		
			double Kp,Ki, Kd;// Dichiarazione valori di Kp (proporzionale), Ki (integrativa), Kd (derivativa) 
		cout<<"Fornire valore di Kp da utilizzare"<<endl;
		cin>>Kp;
     	cout<<"Fornire valore di Ki da utilizzare"<<endl;
		cin>>Ki;
		cout<<"Fornire valore di Kd da utilizzare"<<endl;
		cin>>Kd;
		
		status= PID_(Kp,Ki,Kd,Tsetpoint);
		if(status== ErrorePid)
			cout<<"Errore: Superata Temperatura Massima "<<endl;
		
			break;
		case 3:
			
			TaskHandle lettura;
			DAQmxCreateTask("", &lettura);
			DAQmxStartTask(lettura);
			DAQmxCreateAIVoltageChan(lettura,"Dev1/ai0","",DAQmx_Val_Cfg_Default,0.0,100e-3,DAQmx_Val_Volts,"");
			DAQmxReadAnalogScalarF64(lettura,10.0,&fValue,0);
			temperatura = conv_temperatura(fValue*1000);
			cout<<temperatura<<endl;
			DAQmxStopTask(lettura);
			DAQmxClearTask(lettura);

			break;
		case 4:
		break;

		default:
			cout<<"Prego ridare comando, grazie"<< endl;
			break;
	}

} while (opzione!=4);

	return 0;
