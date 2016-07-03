#include <string>
#include "Objeto.hpp"

using namespace std;

Objeto::Objeto(string meu_id, string meu_simbolo, int x_ini, int y_ini){
	id = meu_id;
	simbolo = meu_simbolo;
	x = x_ini;
	y = y_ini;
}

void Objeto::atualizar(int & x, int & y){
	x = this->x;
	y = this->y;
}