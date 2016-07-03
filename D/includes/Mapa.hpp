#ifndef MAPA_HPP
#define MAPA_HPP

#include <string>
#include <map>
#include <iostream>
#include "Objeto.hpp"

using namespace std;

class Mapa
{
public:
	Mapa(){}

	Mapa(int m, int n);

	~Mapa();

	string representacao() const;

	void inserir_obstaculo(int, int);

	void inserir_retangulo(int, int, int, int);

	void inserir_objeto(Objeto * obj);

	void atualizar_mapa();

private:
	int * mapa;
	map<string, Objeto*> objs;
	int m,n;
	string repr;
	void construir_representacao();

	void desenhar_objeto(Objeto*, int, int);

	void remover_objeto(int, int);
};

std::ostream &operator<<(std::ostream &, Mapa const &);

#endif /* MAPA_HPP */