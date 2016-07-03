#ifndef OBJETO_HPP
#define OBJETO_HPP

#include <string>

using namespace std;

class Objeto{
public:
	Objeto(string meu_id, string meu_simbolo, int x_ini, int y_ini);
	
	string get_id() const{return id;}
	
	int get_x() const{return x;}
	
	int get_y() const{return y;}

	string get_simbolo() const{return simbolo;}

	virtual void atualizar(int & x, int & y);
	
protected:
	string id,simbolo;
	int x, y;
};

#endif /* OBJETO_HPP */