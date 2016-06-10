#ifndef COORDENADAS_HPP
#define COORDENADAS_HPP

using namespace std;

class Coordenadas{
public:
	Coordenadas();

	Coordenadas(int,int);

	int get_x() const{return x;}

	int get_y() const{return y;}
private:
	int x,y;
};

#endif /* COORDENADAS_HPP */