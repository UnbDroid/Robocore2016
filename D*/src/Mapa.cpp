#include "Mapa.hpp"
#include "Objeto.hpp"
#include <string>
#include <iostream>
#include <map>

using namespace std;

Mapa::Mapa(int m, int n){
	mapa = new int [m*n];
	this->m = m;
	this->n = n;
	for (int i = 0; i < m*n; ++i){
		mapa[i] = 0;
	}
	construir_representacao();
}

Mapa::~Mapa(){
	delete[] mapa;
}

string Mapa::representacao() const{
	return repr;
}

void Mapa::inserir_obstaculo(int m, int n){
	if(m >= this->m || n >= this->n || m < 0 || n < 0)
		return;
	mapa[n*this->m+m] = 1;
	repr.replace(4*(n*this->m+m)+2,1,"\u25A0",2,2);
}

void Mapa::inserir_retangulo(int m_inicial, int n_inicial, int m_final, int n_final){
	bool cond1 = (m_inicial >= m || n_inicial >= n || m < 0 || n < 0);
	bool cond2 = (m_final >= m || n_final >= n);
	bool cond3 = (m_inicial > m_final || n_inicial > n_final);
	if(cond1 || cond2 || cond3)
		return;

	for (int i = m_inicial; i <= m_final; ++i){
		for (int j = n_inicial; j <= n_final; ++j){
			inserir_obstaculo(i,j);
		}
	}
}

void Mapa::inserir_objeto(Objeto * obj){
	if ( objs.find(obj->get_id()) == objs.end() ) {
	  objs.insert(pair<string, Objeto*>(obj->get_id(),obj));
	}
}

void Mapa::atualizar_mapa(){
	int x,y;
	for (map<string, Objeto*>::iterator it=objs.begin(); it!=objs.end(); ++it){
   		Objeto* obj = it->second;
   		remover_objeto(obj->get_x(),obj->get_y());
   		obj->atualizar(x,y);
   		desenhar_objeto(obj,x,y);
	}
}

void Mapa::construir_representacao(){
	repr = string();
	for (int i = 0; i < n; ++i){
		for (int j = 0; j < m; ++j){
			if(mapa[i*m+j]==0){
				repr += "\u25A1";
			}else if(mapa[i*m+j]==1){
				repr += "\u25A0";
			}else if(mapa[i*m+j]==2){
				repr += "\u25A3";
			}
			if(j<m-1)
				repr+=" ";
		}
		repr+='\n';
	}
}

void Mapa::desenhar_objeto(Objeto* obj, int x, int y){
	if(x >= m || y >= n || x < 0 || y < 0)
		return;
	mapa[y*m+x] = 3;
	repr.replace(4*(y*m+x)+2,1,obj->get_simbolo(),2,2);
}

void Mapa::remover_objeto(int x, int y){
	if(x >= m || y >= n || x < 0 || y < 0)
		return;
	mapa[y*m+x] = 3;
	repr.replace(4*(y*m+x)+2,1,"\u25A1",2,2);
}

std::ostream &operator<<(std::ostream &os, Mapa const &m) { 
    return os << m.representacao();
}