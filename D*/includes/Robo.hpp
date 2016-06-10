#ifndef ROBO_HPP
#define ROBO_HPP

#include "Objeto.hpp"
#include "Mapa.hpp"
#include "Estado.hpp"

using namespace std;

class Robo: public Objeto
{
public:
	Robo(string nome_robo, Mapa & mapa_inicial,int pos_x_inicial ,int pos_y_inicial, string simbolo="\u25A3");
	void atualizar(int &, int &);
	void definir_velocidade(int v_x, int v_y);
private:
	Mapa & meu_mapa;
	Estado estado_atual;
	int v_x,v_y;
	void calcular_estado_atual();
};

#endif /* ROBO_HPP */