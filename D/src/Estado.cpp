#include "Estado.hpp"
#include "Coordenadas.hpp"

Estado::Estado(){
	coord = Coordenadas();
}

Estado::Estado(int pos_x, int pos_y){
	coord = Coordenadas(pos_x, pos_y);
}

Estado::Estado(Coordenadas coord){
	this->coord = coord;
}