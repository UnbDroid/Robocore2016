#include <string>
#include "Objeto.hpp"
#include "Mapa.hpp"
#include "Estado.hpp"
#include "Robo.hpp"

Robo::Robo(string nome_robo, Mapa & mapa_inicial,int pos_x_inicial ,int pos_y_inicial, string simbolo):
	Objeto(nome_robo, simbolo, pos_x_inicial, pos_y_inicial),meu_mapa(mapa_inicial){
	estado_atual = Estado(pos_x_inicial, pos_y_inicial);
	meu_mapa.inserir_objeto(this);
	definir_velocidade(0,0);
}

void Robo::atualizar(int & x, int & y){
	calcular_estado_atual();
	x = estado_atual.get_coord().get_x();
	y = estado_atual.get_coord().get_y();
	this->x=x;
	this->y=y;
}

void Robo::definir_velocidade(int v_x, int v_y){
	this->v_x = v_x;
	this->v_y = v_y;
}

void Robo::calcular_estado_atual(){
	int x,y;
	x = estado_atual.get_coord().get_x();
	y = estado_atual.get_coord().get_y();
	estado_atual = Estado(x+v_x,y+v_y);
}