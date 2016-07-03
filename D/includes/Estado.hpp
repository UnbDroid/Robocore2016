#ifndef ESTADO_HPP
#define ESTADO_HPP

#include "Coordenadas.hpp"

using namespace std;

class Estado{
public:
	Estado();

	Estado(int, int);

	Estado(Coordenadas);

	Coordenadas get_coord() const{return coord;}
private:
	Coordenadas coord;
};

#endif /* ESTADO_HPP */