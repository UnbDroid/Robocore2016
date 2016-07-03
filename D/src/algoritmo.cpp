#include <iostream>
#include <string>
#include <cstdlib>
#include <unistd.h>

#include "Objeto.hpp"
#include "Mapa.hpp"
#include "Estado.hpp"
#include "Robo.hpp"


using namespace std;

// class Sensor{
// public:
// 	Sensor(Mapa & mapa, Robo & robo):meu_mapa(mapa),meu_robo(robo){}
// 	//int ** ler_sensor(){
// 	//	return;
// 	//}

// private:
// 	Mapa & meu_mapa;
// 	Robo & meu_robo;
// };

int main(int argc, char const *argv[])
{
	Mapa mapa_robo(10,10);
	mapa_robo.inserir_retangulo(1,2,1,3);
	//Robo bruce("Bruce", mapa_robo, 0,0);
	mapa_robo.atualizar_mapa();
	cout << mapa_robo << endl;
	//bruce.definir_velocidade(1,0);
	for (int i = 0; i < 4; ++i){
		system("clear");
		mapa_robo.atualizar_mapa();
		cout << mapa_robo << endl;
		sleep(1);
	}
	//bruce.definir_velocidade(0,1);
	for (int i = 0; i < 4; ++i){
		system("clear");
		mapa_robo.atualizar_mapa();
		cout << mapa_robo << endl;
		sleep(1);
	}
	return 0;
}